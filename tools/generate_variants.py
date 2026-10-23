#!/usr/bin/env python3
"""
Generate the MuJoCo models of the G1 hardware variants from the Unitree URDF files of g1_description.

The model of a variant is obtained from a template model of the same family (23dof, 29dof, 29dof_no_hands,
29dof_revo2) in which every quantity that depends on the hardware revision is recomputed from the URDF:
  - body positions/orientations (joint origins)
  - inertial of each body, including the fixed links merged into it (same as MuJoCo fusestatic)
  - meshes (mesh files of the links, geoms placed from the URDF visual/collision origins, removed if the link is gone,
    collision meshes of the URDF added when the template has none for that link)
  - joint ranges, actuator force ranges, motor control ranges (URDF effort)
  - joints that are fixed in the URDF (locked waist): joint, actuator and sensors removed
  - IMU sites (imu_in_pelvis -> site "imu", imu_in_torso -> site "secondary_imu")
Everything else (defaults, joint damping/armature/friction, collision geoms, sensors, extra bodies such as the
Revo2 hands) is taken from the template.

Usage: generate_variants.py <g1_description dir> <g1_mj_description dir>
"""
import copy
import os
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np

# URDF variant -> (template family, mode_machine)
VARIANTS = {
    "23dof_rev_1_0": 4,
    "29dof_rev_1_0": 5,
    "29dof_lock_waist_rev_1_0": 6,
    "23dof_mode_10": 10,
    "29dof_mode_11": 11,
    "29dof_mode_12": 12,
    "29dof_mode_13": 13,
    "29dof_mode_14": 14,
    "29dof_mode_15": 15,
    "29dof_mode_16": 16,
    "29dof_mode_18": 18,
}
SITE_LINKS = {"imu": "imu_in_pelvis", "secondary_imu": "imu_in_torso"}
# Fixed links of the deprecated models that do not exist in every hardware revision
OPTIONAL_LINKS = {"pelvis_contour_link", "waist_support_link", "waist_yaw_fixed_link", "waist_yaw_link"}


# ----------------------------------------------------------------------------------------------- math helpers
def rpy_to_mat(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def mat_to_quat(R):
    """Rotation matrix -> quaternion (w, x, y, z), w >= 0"""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    return q if q[0] >= 0 else -q


ORIENTATION_ATTRS = ("quat", "euler", "axisangle", "xyaxes", "zaxis")


def set_orientation(el, q):
    """Set the orientation of a MJCF element as a quaternion (removing any other orientation specification)"""
    for k in ORIENTATION_ATTRS:
        el.attrib.pop(k, None)
    if not np.allclose(q, [1, 0, 0, 0], atol=1e-9):
        el.set("quat", fmt(q))


def origin(el):
    """URDF <origin> of an element -> (xyz, R)"""
    o = el.find("origin") if el is not None else None
    xyz = (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()
    rpy = (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()
    return np.array([float(x) for x in xyz]), rpy_to_mat([float(x) for x in rpy])


def is_visual(g):
    return g.get("contype") == "0" and g.get("conaffinity") == "0"


def place_geom(g, p, R):
    if np.allclose(p, 0, atol=1e-12):
        g.attrib.pop("pos", None)
    else:
        g.set("pos", fmt(p))
    set_orientation(g, mat_to_quat(R))


def fmt(v):
    return " ".join(f"{x:.9g}" if abs(x) > 1e-12 else "0" for x in v)


# ----------------------------------------------------------------------------------------------- URDF
class Urdf:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.links = {}
        for l in root.findall("link"):
            inertial = l.find("inertial")
            mesh = l.find("visual/geometry/mesh")
            info = {"mesh": os.path.basename(mesh.get("filename")) if mesh is not None else None, "mass": 0.0,
                    "visual": origin(l.find("visual")),
                    "collision": [origin(c) for c in l.findall("collision") if c.find("geometry/mesh") is not None
                                  and os.path.basename(c.find("geometry/mesh").get("filename")) == (
                                      os.path.basename(mesh.get("filename")) if mesh is not None else None)]}
            if inertial is not None:
                o = inertial.find("origin")
                I = inertial.find("inertia")
                g = lambda k: float(I.get(k, 0))
                info.update(
                    mass=float(inertial.find("mass").get("value")),
                    com=np.array([float(x) for x in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]),
                    R=rpy_to_mat([float(x) for x in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]),
                    I=np.array([[g("ixx"), g("ixy"), g("ixz")], [g("ixy"), g("iyy"), g("iyz")], [g("ixz"), g("iyz"), g("izz")]]),
                )
            self.links[l.get("name")] = info
        self.joints = {}
        self.parent_joint = {}
        self.children = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            lim = j.find("limit")
            info = dict(
                name=j.get("name"),
                type=j.get("type"),
                parent=j.find("parent").get("link"),
                child=j.find("child").get("link"),
                xyz=np.array([float(x) for x in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]),
                R=rpy_to_mat([float(x) for x in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]),
                lower=float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
                upper=float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
                effort=float(lim.get("effort")) if lim is not None and lim.get("effort") else None,
            )
            self.joints[info["name"]] = info
            self.parent_joint[info["child"]] = info
            self.children.setdefault(info["parent"], []).append(info)

    def fixed_descendants(self, link, stop):
        """Links attached to `link` through fixed joints only, not crossing links in `stop`, with their pose in `link`"""
        out = []
        def rec(l, p, R):
            for j in self.children.get(l, []):
                if j["type"] != "fixed" or j["child"] in stop:
                    continue
                pc, Rc = p + R @ j["xyz"], R @ j["R"]
                out.append((j["child"], pc, Rc))
                rec(j["child"], pc, Rc)
        rec(link, np.zeros(3), np.eye(3))
        return out


def composite_inertia(urdf, items):
    """items: [(link, p, R)] poses in the body frame -> (mass, com, quat, diaginertia)"""
    m_tot, c_tot = 0.0, np.zeros(3)
    assert any(urdf.links[l]["mass"] > 0 for l, _, _ in items), items
    parts = []
    for link, p, R in items:
        L = urdf.links[link]
        if L["mass"] <= 0:
            continue
        c = p + R @ L["com"]
        Rin = R @ L["R"]
        parts.append((L["mass"], c, Rin @ L["I"] @ Rin.T))
        m_tot += L["mass"]
        c_tot += L["mass"] * c
    c_tot /= m_tot
    I = np.zeros((3, 3))
    for m, c, Ic in parts:
        d = c - c_tot
        I += Ic + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    w, V = np.linalg.eigh(I)
    order = np.argsort(w)[::-1]
    w, V = w[order], V[:, order]
    if np.linalg.det(V) < 0:
        V[:, 2] *= -1
    return m_tot, c_tot, mat_to_quat(V), w


# ----------------------------------------------------------------------------------------------- generation
def load_xml(path):
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.parse(path, parser)


def generate(template_path, urdf, model_name, out_path):
    tree = load_xml(template_path)
    root = tree.getroot()
    root.set("model", model_name)
    worldbody = root.find("worldbody")
    bodies = {b.get("name"): b for b in worldbody.iter("body")}
    parent_of = {c: p for p in worldbody.iter() for c in p}
    urdf_bodies = [b for b in bodies if b in urdf.links]
    stop = set(urdf_bodies)
    removed_joints = []
    removed_links = set()

    # mesh name -> link name (mesh assets are named after the links)
    assets = root.find("asset")
    for mesh in list(assets.findall("mesh")):
        name = mesh.get("name")
        if name in urdf.links:
            if urdf.links[name]["mesh"]:
                mesh.set("file", urdf.links[name]["mesh"])
        elif name in OPTIONAL_LINKS:
            # link of the G1 that does not exist in this variant (e.g. pelvis_contour_link, waist_support_link)
            assets.remove(mesh)
            removed_links.add(name)

    for name in urdf_bodies:
        body = bodies[name]
        # body pose from the URDF joint
        if name in urdf.parent_joint and parent_of[body].tag == "body":
            j = urdf.parent_joint[name]
            body.set("pos", fmt(j["xyz"]))
            set_orientation(body, mat_to_quat(j["R"]))
            jel = body.find("joint")
            if j["type"] == "fixed":
                if jel is not None:
                    body.remove(jel)
                    removed_joints.append(j["name"])
            elif jel is not None:
                jel.set("range", fmt([j["lower"], j["upper"]]))
                jel.set("actuatorfrcrange", fmt([-j["effort"], j["effort"]]))
        # merged fixed links
        merged = urdf.fixed_descendants(name, stop)
        merged_pose = {l: (p, R) for l, p, R in merged}
        # inertial
        inertial = body.find("inertial")
        if inertial is not None:
            m, c, q, d = composite_inertia(urdf, [(name, np.zeros(3), np.eye(3))] + merged)
            inertial.attrib.clear()
            inertial.set("pos", fmt(c))
            inertial.set("quat", fmt(q))
            inertial.set("mass", f"{m:.9g}")
            inertial.set("diaginertia", fmt(d))
        # mesh geoms of the body link and of the merged links, placed from the URDF visual / collision origins
        link_pose = {name: (np.zeros(3), np.eye(3)), **merged_pose}
        for g in list(body.findall("geom")):
            mesh = g.get("mesh")
            if mesh is None:
                continue
            if mesh in removed_links:
                body.remove(g)
                continue
            if mesh not in link_pose:
                continue
            p, R = link_pose[mesh]
            L = urdf.links[mesh]
            if is_visual(g) or not L["collision"]:
                vp, vR = L["visual"]
            else:
                vp, vR = L["collision"][0]
            place_geom(g, p + R @ vp, R @ vR)
        # collision meshes of the URDF that the template does not have (e.g. rubber hands)
        for link, (p, R) in link_pose.items():
            L = urdf.links[link]
            if not L["collision"] or not L["mesh"]:
                continue
            geoms = [g for g in body.findall("geom") if g.get("mesh") == link]
            vis = next((g for g in geoms if is_visual(g)), None)
            if vis is None or any(not is_visual(g) for g in geoms):
                continue
            col = ET.Element("geom", {"type": "mesh", "rgba": vis.get("rgba", "0.7 0.7 0.7 1"), "mesh": link})
            cp, cR = L["collision"][0]
            place_geom(col, p + R @ cp, R @ cR)
            body.insert(list(body).index(vis) + 1, col)
    # IMU sites, placed in the body the IMU link is merged into
    for site_name, link in SITE_LINKS.items():
        site = next((s for s in worldbody.iter("site") if s.get("name") == site_name), None)
        if site is None or link not in urdf.links:
            continue
        for name in urdf_bodies:
            pose = {l: (p, R) for l, p, R in urdf.fixed_descendants(name, stop)}
            if link in pose:
                parent_of[site].remove(site)
                # keep sites first in the body (before inertial) for readability
                bodies[name].insert(0, site)
                site.set("pos", fmt(pose[link][0]))
                set_orientation(site, mat_to_quat(pose[link][1]))
                break

    # actuators and sensors
    actuator = root.find("actuator")
    for a in list(actuator):
        jn = a.get("joint")
        if jn in removed_joints:
            actuator.remove(a)
        elif jn in urdf.joints and a.tag == "motor" and urdf.joints[jn]["effort"]:
            e = urdf.joints[jn]["effort"]
            a.set("ctrlrange", fmt([-e, e]))
    sensor = root.find("sensor")
    for s in list(sensor):
        if s.get("joint") in removed_joints:
            sensor.remove(s)

    ET.indent(tree, space="  ")
    header = (f"<!-- Generated by tools/generate_variants.py from {os.path.basename(template_path)} and the Unitree URDF "
              f"{model_name}.urdf of g1_description. Do not edit by hand. -->\n")
    with open(out_path, "w") as f:
        f.write(header)
        tree.write(f, encoding="unicode")
        f.write("\n")
    return removed_joints


def main():
    desc, mj = sys.argv[1], sys.argv[2]
    xml_dir, gains_dir, mesh_dir = f"{mj}/xml", f"{mj}/pdgains", f"{mj}/meshes"
    generated = []
    for variant in VARIANTS:
        dof = variant[:5]
        kinds = [("", f"g1_{dof}")]
        if dof == "29dof":
            kinds += [("_no_hands", "g1_29dof_no_hands"), ("_revo2", "g1_29dof_revo2")]
        for suffix, template in kinds:
            urdf_name = f"g1_{variant}" + ("_no_hands" if suffix else "")
            model = f"g1_{variant}{suffix}"
            urdf = Urdf(f"{desc}/urdf/{urdf_name}.urdf")
            removed = generate(f"{xml_dir}/{template}.xml", urdf, model, f"{xml_dir}/{model}.xml")
            # PD gains: same as the template family, without the removed (locked) joints
            os.makedirs(f"{gains_dir}/{model}", exist_ok=True)
            lines = open(f"{gains_dir}/{template}/PDgains_sim.dat").read().split("\n")
            lines = [l for l in lines if l.strip()]
            if removed:
                # PD gains are in actuator order: find the indices of the removed actuators in the template
                tmpl_act = [a.get("joint") for a in load_xml(f"{xml_dir}/{template}.xml").getroot().find("actuator")]
                lines = [l for i, l in enumerate(lines) if tmpl_act[i] not in removed]
            open(f"{gains_dir}/{model}/PDgains_sim.dat", "w").write("\n".join(lines))
            # meshes
            for l in urdf.links.values():
                if l["mesh"] and not os.path.exists(f"{mesh_dir}/{l['mesh']}"):
                    shutil.copy(f"{desc}/meshes/{l['mesh']}", f"{mesh_dir}/{l['mesh']}")
                    print("copied mesh", l["mesh"])
            generated.append((model, removed))
    for model, removed in generated:
        print(f"generated {model}" + (f" (locked: {', '.join(removed)})" if removed else ""))


if __name__ == "__main__":
    main()
