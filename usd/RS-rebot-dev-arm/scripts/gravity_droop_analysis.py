"""Gravity-compensation impact of the PR#3 mass update (b094da6 vs c2eba19).

For each revolute arm joint, computes over a grid of in-limit poses:
  - worst-case gravity torque |tau_g| about the joint axis
  - static droop under the validated PD gains (drive stiffness is authored
    per-degree in USD: K_rad = K_stored * 180/pi)
  - accumulated distal inertia about the joint axis (what the Gain Tuner's
    compute_joints_accumulated_inertia uses as m_eq for a world-welded base)
  - resulting natural frequency / damping ratio with the current gains,
    using the same formulas as isaacsim.robot_setup.gain_tuner
    .gain_tuner_drive_math (force drives).

Run: .demo/bin/python gravity_droop_analysis.py
Writes evidence/gravity_droop.json and prints a comparison table.
"""

import json
import math
import re
import subprocess
import xml.etree.ElementTree as ET
from itertools import product
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
URDF = REPO / "urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf"
OLD_REV = "c2eba19"
EVIDENCE = Path(__file__).resolve().parent.parent / "evidence"

G = np.array([0.0, 0.0, -9.81])
DEG_TO_RAD = math.pi / 180.0

# validated gains (stored per-degree stiffness/damping, force drives)
GAINS = {
    "joint1": (500.0, 60.0),
    "joint2": (1500.0, 96.0),
    "joint3": (1000.0, 76.0),
    "joint4": (150.0, 18.0),
    "joint5": (80.0, 10.0),
    "joint6": (50.0, 7.0),
}

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def axis_angle(axis, q):
    axis = axis / np.linalg.norm(axis)
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0],
    ])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)


def parse_urdf(text):
    root = ET.fromstring(text)
    links = {}
    for l in root.findall("link"):
        inertial = l.find("inertial")
        if inertial is None:
            continue
        o = inertial.find("origin")
        xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0")).split()]) if o is not None else np.zeros(3)
        rpy = np.array([float(v) for v in (o.get("rpy", "0 0 0")).split()]) if o is not None else np.zeros(3)
        m = float(inertial.find("mass").get("value"))
        it = inertial.find("inertia")
        I = np.array([
            [float(it.get("ixx")), float(it.get("ixy")), float(it.get("ixz"))],
            [float(it.get("ixy")), float(it.get("iyy")), float(it.get("iyz"))],
            [float(it.get("ixz")), float(it.get("iyz")), float(it.get("izz"))],
        ])
        links[l.get("name")] = {"m": m, "com_xyz": xyz, "com_rpy": rpy, "I": I}
    joints = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = np.array([float(v) for v in (o.get("xyz", "0 0 0")).split()]) if o is not None else np.zeros(3)
        rpy = np.array([float(v) for v in (o.get("rpy", "0 0 0")).split()]) if o is not None else np.zeros(3)
        ax = j.find("axis")
        axis = np.array([float(v) for v in (ax.get("xyz", "1 0 0")).split()]) if ax is not None else np.array([1.0, 0, 0])
        lim = j.find("limit")
        lo = float(lim.get("lower", 0)) if lim is not None else 0.0
        hi = float(lim.get("upper", 0)) if lim is not None else 0.0
        joints[j.get("name")] = {
            "type": j.get("type"),
            "parent": j.find("parent").get("link"),
            "child": j.find("child").get("link"),
            "xyz": xyz, "rpy": rpy, "axis": axis, "lower": lo, "upper": hi,
        }
    return links, joints


def chain_order(joints):
    # serial chain from base_link following revolute arm joints
    order = []
    parent = "base_link"
    while True:
        nxt = [n for n, j in joints.items() if j["parent"] == parent and n in ARM_JOINTS]
        if not nxt:
            break
        order.append(nxt[0])
        parent = joints[nxt[0]]["child"]
    return order


def fk(links, joints, order, q):
    """Returns per-joint world (origin, axis) and per-link world CoM/rot for
    all links distal to each joint (incl. gripper via fixed joints)."""
    # walk the full tree once
    T = {"base_link": (np.eye(3), np.zeros(3))}
    qmap = dict(zip(order, q))
    joint_world = {}
    pending = list(joints.items())
    while pending:
        remaining = []
        for name, j in pending:
            if j["parent"] not in T:
                remaining.append((name, j))
                continue
            Rp, pp = T[j["parent"]]
            Rj = Rp @ rpy_to_mat(*j["rpy"])
            pj = pp + Rp @ j["xyz"]
            if j["type"] == "revolute" and name in qmap:
                axis_w = Rj @ j["axis"]
                joint_world[name] = (pj, axis_w)
                Rc = Rj @ axis_angle(j["axis"], qmap[name])
            elif j["type"] == "prismatic":
                Rc = Rj  # grippers at closed position
            else:
                Rc = Rj
            T[j["child"]] = (Rc, pj)
        if len(remaining) == len(pending):
            break
        pending = remaining
    coms = {}
    for lname, l in links.items():
        if lname not in T:
            continue
        R, p = T[lname]
        coms[lname] = (p + R @ l["com_xyz"], R @ rpy_to_mat(*l["com_rpy"]))
    return joint_world, coms


def distal_links(joints, joint_name):
    out = []
    frontier = [joints[joint_name]["child"]]
    while frontier:
        l = frontier.pop()
        out.append(l)
        frontier += [j["child"] for j in joints.values() if j["parent"] == l]
    return out


def analyze(links, joints, label):
    order = chain_order(joints)
    grids = []
    for n in order:
        j = joints[n]
        grids.append(np.linspace(j["lower"], j["upper"], 5))
    mid_pose = [float(np.mean([joints[n]["lower"], joints[n]["upper"]])) for n in order]
    worst_tau = {n: 0.0 for n in order}
    # default to mid pose: a joint whose axis stays parallel to gravity has
    # tau == 0 over the whole grid and never updates its worst pose
    worst_pose = {n: mid_pose for n in order}
    for q in product(*grids):
        jw, coms = fk(links, joints, order, q)
        for n in order:
            pj, axis = jw[n]
            tau = 0.0
            for l in distal_links(joints, n):
                if l not in coms or l not in links:
                    continue
                c, _ = coms[l]
                tau += np.dot(np.cross(c - pj, links[l]["m"] * G), axis)
            if abs(tau) > abs(worst_tau[n]):
                worst_tau[n] = tau
                worst_pose[n] = [round(float(v), 3) for v in q]

    # accumulated distal inertia about the joint axis at the worst pose
    m_eq = {}
    for n in order:
        q = worst_pose[n]
        jw, coms = fk(links, joints, order, q)
        pj, axis = jw[n]
        I_total = 0.0
        for l in distal_links(joints, n):
            if l not in coms:
                continue
            c, Rc = coms[l]
            I_world = Rc @ links[l]["I"] @ Rc.T
            d = c - pj
            # parallel axis: I_axis = a·I_com·a + m(|d|^2 - (d·a)^2)
            I_total += axis @ I_world @ axis + links[l]["m"] * (d @ d - (d @ axis) ** 2)
        m_eq[n] = I_total

    rows = {}
    for n in order:
        k_stored, d_stored = GAINS[n]
        k_rad = k_stored / DEG_TO_RAD           # N*m/rad (USD stores per-degree)
        d_rad = d_stored / DEG_TO_RAD
        droop = abs(worst_tau[n]) / k_rad        # rad
        fn = math.sqrt(k_rad / m_eq[n]) / (2 * math.pi) if m_eq[n] > 0 else float("inf")
        zeta = d_rad / (2 * math.sqrt(m_eq[n] * k_rad)) if m_eq[n] > 0 else float("inf")
        rows[n] = {
            "worst_tau_Nm": round(float(worst_tau[n]), 4),
            "worst_pose": worst_pose[n],
            "droop_rad": float(f"{droop:.3e}"),
            "droop_deg": float(f"{math.degrees(droop):.3e}"),
            "m_eq_kgm2": float(f"{m_eq[n]:.5f}"),
            "f_n_hz": round(fn, 2),
            "zeta": round(zeta, 3),
        }
    return rows


def main():
    new_text = URDF.read_text()
    old_text = subprocess.run(
        ["git", "-C", str(REPO), "show", f"{OLD_REV}:urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf"],
        capture_output=True, text=True, check=True,
    ).stdout

    results = {}
    for label, text in (("old_masses_" + OLD_REV, old_text), ("new_masses_b094da6", new_text)):
        links, joints = parse_urdf(text)
        results[label] = analyze(links, joints, label)

    old, new = results["old_masses_" + OLD_REV], results["new_masses_b094da6"]
    hdr = f"{'joint':8s} {'tau_old':>9s} {'tau_new':>9s} {'droop_old_deg':>14s} {'droop_new_deg':>14s} {'m_eq old':>9s} {'m_eq new':>9s} {'f_n o/n':>13s} {'zeta o/n':>13s}"
    print(hdr)
    for n in ARM_JOINTS:
        o, w = old[n], new[n]
        print(
            f"{n:8s} {o['worst_tau_Nm']:9.3f} {w['worst_tau_Nm']:9.3f}"
            f" {o['droop_deg']:14.3e} {w['droop_deg']:14.3e}"
            f" {o['m_eq_kgm2']:9.4f} {w['m_eq_kgm2']:9.4f}"
            f" {o['f_n_hz']:6.2f}/{w['f_n_hz']:<6.2f} {o['zeta']:6.3f}/{w['zeta']:<6.3f}"
        )

    EVIDENCE.mkdir(exist_ok=True)
    out = EVIDENCE / "gravity_droop.json"
    json.dump({"gains_stored_per_deg": GAINS, "g": -9.81, "grid": "5 points/joint over limits", "results": results}, open(out, "w"), indent=1)
    print("\nWROTE", out)


if __name__ == "__main__":
    main()
