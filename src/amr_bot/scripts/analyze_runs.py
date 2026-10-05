#!/usr/bin/env python3
"""Plot and score one path_logger run: planned path vs actual motion.

usage: analyze_run.py runs/baseline_002 [--map maps/market.yaml] [--compare runs/other_001]
                      [--out DIR] [--tail 1.0]
Writes <out>/<run>_xy.png, _errors.png, _vel.png, _summary.txt. Needs only numpy + matplotlib.
"""
import argparse
import csv
import os
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

TOL_XY, TOL_YAW = 0.25, 0.25     # nav2 general_goal_checker (config/nav2_params.yaml)


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def read_csv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def load_plans(path):
    """Distinct plans in order. The controller re-sends each plan every cycle, so drop repeats."""
    d = read_csv(path)
    cut = np.flatnonzero(np.diff(d["plan_id"])) + 1
    plans, prev = [], None
    for idx in np.split(np.arange(len(d["plan_id"])), cut):
        xy = np.column_stack([d["x"][idx], d["y"][idx]])
        key = (len(xy), *np.round(xy[0], 3), *np.round(xy[-1], 3))
        if key != prev:
            plans.append((float(d["t_recv"][idx[0]]), xy))
            prev = key
    return plans


def load_run(prefix):
    prefix = re.sub(r"_(traj|plans|goal)\.csv$", "", prefix)
    with open(prefix + "_goal.csv") as f:
        goal = next(csv.DictReader(f))
    return prefix, read_csv(prefix + "_traj.csv"), goal, load_plans(prefix + "_plans.csv")


def read_map(yaml_path):
    meta = {}
    for line in open(yaml_path):
        k, _, v = line.partition(":")
        meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    ox, oy = [float(s) for s in meta["origin"].strip("[]").split(",")[:2]]
    data = open(os.path.join(os.path.dirname(yaml_path), meta["image"]), "rb").read()
    tok, pos = [], 0                                   # P5 header: magic, w, h, maxval
    while len(tok) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos)
            continue
        s = pos
        while not data[pos:pos + 1].isspace():
            pos += 1
        tok.append(data[s:pos])
    w, h = int(tok[1]), int(tok[2])
    img = np.frombuffer(data, np.uint8, w * h, pos + 1).reshape(h, w)
    return np.flipud(img), (ox, ox + w * res, oy, oy + h * res)


def to_polyline(p, xy):
    """Per point of p (N,2): distance, signed cross-track (left of path = +), path tangent angle."""
    a, ab = xy[:-1], np.diff(xy, axis=0)
    L2 = np.maximum((ab ** 2).sum(1), 1e-12)
    u = np.clip(((p[:, None] - a[None]) * ab[None]).sum(2) / L2[None], 0, 1)
    proj = a[None] + u[..., None] * ab[None]
    d = np.linalg.norm(p[:, None] - proj, axis=2)
    j, n = d.argmin(1), np.arange(len(p))
    cross = ab[j, 0] * (p[:, 1] - proj[n, j, 1]) - ab[j, 1] * (p[:, 0] - proj[n, j, 0])
    return d[n, j], np.sign(cross) * d[n, j], np.arctan2(ab[j, 1], ab[j, 0])


def path_len(xy):
    return float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())


def analyse(prefix, tail):
    prefix, T, goal, plans = load_run(prefix)
    g = dict(x=float(goal["goal_x"]), y=float(goal["goal_y"]), yaw=float(goal["goal_yaw"]))
    t_end = float(goal["t_end"])
    t = T["t"]
    r = dict(prefix=prefix, T=T, goal=g, status=goal["end_status"], t_end=t_end, plans=plans)

    r["d_goal"] = np.hypot(T["x_map"] - g["x"], T["y_map"] - g["y"])
    r["d_goal_gt"] = np.hypot(T["x_gt"] - g["x"], T["y_gt"] - g["y"])
    r["yaw_err"] = wrap(T["yaw_map"] - g["yaw"])
    r["yaw_err_gt"] = wrap(T["yaw_gt"] - g["yaw"])
    r["amcl_gt"] = np.hypot(T["x_map"] - T["x_gt"], T["y_map"] - T["y_gt"])

    ref = plans[0][1]                                  # the first plan = the originally intended route
    pts = np.column_stack([T["x_map"], T["y_map"]])
    _, r["cte"], tang = to_polyline(pts, ref)
    r["head_err"] = wrap(T["yaw_map"] - tang)

    dt = np.maximum(np.diff(t), 1e-3)
    dx, dy, yaw = np.diff(T["x_gt"]), np.diff(T["y_gt"]), T["yaw_gt"][:-1]
    r["tm"] = (t[:-1] + t[1:]) / 2
    r["v_true"] = (dx * np.cos(yaw) + dy * np.sin(yaw)) / dt          # body-frame forward speed
    r["w_true"] = wrap(np.diff(T["yaw_gt"])) / dt
    r["moving"] = np.hypot(dx, dy) / dt > 0.05
    r["yaw_mid"] = yaw

    cmd = np.flatnonzero((np.abs(T["v_cmd"]) > 1e-3) | (np.abs(T["w_cmd"]) > 1e-3))
    r["t_first_cmd"] = t[cmd[0]] if len(cmd) else float("nan")
    r["t_first_plan"] = plans[0][0]
    r["i_ok"] = int(np.argmin(np.abs(t - (t_end - tail))))             # sample when nav2 declared the goal
    r["tail"] = tail
    return r


def metrics(r):
    T, g, m = r["T"], r["goal"], []
    add = lambda k, v: m.append((k, v))
    i, mv = r["i_ok"], r["moving"]
    add("goal (x, y, yaw)", f"({g['x']:.2f}, {g['y']:.2f}, {g['yaw']:.2f})")
    add("end status / duration", f"{r['status']} / {r['t_end']:.1f} s")
    add("time before first plan", f"{r['t_first_plan']:.1f} s")
    add("first motion command at", f"{r['t_first_cmd']:.1f} s")
    add("distinct plans (replans)", f"{len(r['plans'])}")
    ref = r["plans"][0][1]
    trav = path_len(np.column_stack([T["x_map"], T["y_map"]]))
    trav_gt = path_len(np.column_stack([T["x_gt"], T["y_gt"]]))
    add("initial plan length", f"{path_len(ref):.2f} m")
    add("travelled (AMCL / true)", f"{trav:.2f} / {trav_gt:.2f} m")
    add("goal error at success, AMCL frame", f"{r['d_goal'][i]:.2f} m, yaw {r['yaw_err'][i]:+.3f} rad")
    add("goal error at end, AMCL frame", f"{r['d_goal'][-1]:.2f} m, yaw {r['yaw_err'][-1]:+.3f} rad")
    add("goal error at end, TRUE pose*", f"{r['d_goal_gt'][-1]:.2f} m, yaw {r['yaw_err_gt'][-1]:+.3f} rad")
    add("closest approach to goal (AMCL)", f"{r['d_goal'].min():.2f} m")
    add("cross-track vs initial plan", f"rms {np.sqrt(np.mean(r['cte'] ** 2)):.3f}, max {np.abs(r['cte']).max():.3f}, "
                                        f"bias {r['cte'].mean():+.3f} m")
    add("heading error vs path (moving)", f"mean |e| {np.degrees(np.abs(r['head_err'][:-1][mv]).mean()) if mv.any() else float('nan'):.1f} deg")
    ag = r["amcl_gt"]
    add("AMCL - true position error", f"start {ag[0]:.2f}, end {ag[-1]:.2f}, max {ag.max():.2f} m")
    if len(r["plans"]) > 1:
        dev = [np.mean(to_polyline(b[min(5, len(b) - 1):], a)[0]) for (_, a), (_, b) in zip(r["plans"][:-1], r["plans"][1:])
               if len(a) > 1 and len(b) > 5]
        if dev:
            add("plan-to-plan deviation", f"median {np.median(dev):.3f}, max {np.max(dev):.3f} m")
    if mv.any():
        add("mean speed: cmd / true (moving)", f"{T['v_cmd'][:-1][mv].mean():.3f} / {r['v_true'][mv].mean():.3f} m/s")
        for name, sel in (("+x", np.abs(r["yaw_mid"]) < np.radians(30)),
                          ("+y", (r["yaw_mid"] > np.radians(60)) & (r["yaw_mid"] < np.radians(120))),
                          ("-y", (r["yaw_mid"] < -np.radians(60)) & (r["yaw_mid"] > -np.radians(120))),
                          ("-x", np.abs(r["yaw_mid"]) > np.radians(150))):
            s = mv & sel
            if s.sum() >= 5:
                add(f"  facing {name}: true / odom twist", f"{r['v_true'][s].mean():+.3f} / {T['v_odom'][:-1][s].mean():+.3f} m/s (n={s.sum()})")
    notes = []
    if ag[0] > 0.3:
        notes.append(f"AMCL pose differs from the true pose by {ag[0]:.2f} m at the start: the initial pose / localization is wrong, "
                     "so the AMCL-frame numbers above are not trustworthy.")
    if ag.max() > 0.5:
        notes.append(f"AMCL drifted from the true pose by up to {ag.max():.2f} m during the run.")
    if r["t_first_plan"] > 5:
        notes.append(f"No plan for the first {r['t_first_plan']:.0f} s (planner failing, e.g. start inside an obstacle).")
    if r["status"] != "SUCCEEDED":
        notes.append(f"Run ended {r['status']}.")
    return m, notes


def shade_idle(ax, r):
    if r["t_first_plan"] > 1:
        ax.axvspan(0, r["t_first_plan"], color="0.88", zorder=0)


def fig_xy(r, mapinfo, out):
    T, g = r["T"], r["goal"]
    fig, ax = plt.subplots(figsize=(8, 8))
    if mapinfo:
        ax.imshow(mapinfo[0], cmap="gray", extent=mapinfo[1], origin="lower", zorder=0)
    pl = r["plans"]
    for _, xy in pl[1::max(1, len(pl) // 12)]:
        ax.plot(xy[:, 0], xy[:, 1], color="tab:cyan", lw=0.8, alpha=0.5, zorder=2)
    ax.plot([], [], color="tab:cyan", lw=0.8, label=f"replans ({len(pl)} distinct)")
    ax.plot(pl[0][1][:, 0], pl[0][1][:, 1], color="tab:blue", lw=2.5, label="initial plan", zorder=3)
    ax.plot(T["x_map"], T["y_map"], color="tab:orange", lw=1.8, label="actual, AMCL pose (map frame)", zorder=4)
    ax.plot(T["x_gt"], T["y_gt"], color="tab:green", lw=1.5, ls="--", label="actual, true pose (world)", zorder=4)
    ax.plot(T["x_map"][0], T["y_map"][0], "o", color="tab:orange", zorder=5)
    ax.plot(T["x_gt"][0], T["y_gt"][0], "o", color="tab:green", zorder=5)
    ax.plot(T["x_map"][-1], T["y_map"][-1], "s", color="tab:orange", zorder=5)
    ax.plot(T["x_gt"][-1], T["y_gt"][-1], "s", color="tab:green", zorder=5)
    ax.add_patch(plt.Circle((g["x"], g["y"]), TOL_XY, fill=False, color="red", ls=":", zorder=5))
    ax.arrow(g["x"], g["y"], 0.5 * np.cos(g["yaw"]), 0.5 * np.sin(g["yaw"]), color="red", width=0.03, label="goal pose (0.25 m ring)", zorder=6)
    allx = np.concatenate([T["x_map"], T["x_gt"], pl[0][1][:, 0], [g["x"]]])
    ally = np.concatenate([T["y_map"], T["y_gt"], pl[0][1][:, 1], [g["y"]]])
    ax.set_xlim(allx.min() - 1, allx.max() + 1)
    ax.set_ylim(ally.min() - 1, ally.max() + 1)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"{os.path.basename(r['prefix'])}: {r['status']}  (circle = start, square = end)")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def fig_errors(r, out):
    t = r["T"]["t"]
    fig, ax = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    ax[0].plot(t, r["d_goal"], label="AMCL pose")
    ax[0].plot(t, r["d_goal_gt"], ls="--", label="true pose*")
    ax[0].axhline(TOL_XY, color="red", ls=":")
    ax[0].set_ylabel("distance to goal [m]")
    ax[1].plot(t, r["yaw_err"], label="AMCL pose")
    ax[1].plot(t, r["yaw_err_gt"], ls="--", label="true pose*")
    ax[1].axhspan(-TOL_YAW, TOL_YAW, color="red", alpha=0.12)
    ax[1].set_ylabel("yaw error to goal [rad]")
    ax[2].plot(t, r["cte"], color="tab:purple")
    ax[2].axhline(0, color="k", lw=0.5)
    ax[2].set_ylabel("cross-track vs\ninitial plan [m] (left +)")
    ax[3].plot(t, r["amcl_gt"], color="tab:red")
    ax[3].set_ylabel("|AMCL - true| position [m]")
    ax[3].set_xlabel("time [s]")
    for a in ax:
        shade_idle(a, r)
        a.axvline(r["t_end"] - r["tail"], color="green", ls="-.", lw=1)
        a.grid(alpha=0.3)
    ax[0].legend(fontsize=8, loc="upper right")
    ax[0].set_title(f"{os.path.basename(r['prefix'])}  (grey = no plan yet, green line = nav2 declared {r['status']}; "
                    "* assumes map frame = world frame)", fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def fig_vel(r, out):
    T, t = r["T"], r["T"]["t"]
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    ax[0].plot(t, T["v_cmd_nav"], label="cmd from controller", lw=1)
    ax[0].plot(t, T["v_cmd"], label="cmd after smoother (to robot)", lw=1)
    ax[0].plot(t, T["v_odom"], label="reported /odom twist", lw=1.2)
    ax[0].plot(r["tm"], r["v_true"], label="true body speed (from true pose)", lw=1.2)
    ax[0].set_ylabel("linear v [m/s]")
    ax[1].plot(t, T["w_cmd_nav"], lw=1, label="cmd from controller")
    ax[1].plot(t, T["w_cmd"], lw=1, label="cmd to robot")
    ax[1].plot(t, T["w_odom"], lw=1.2, label="reported /odom twist")
    ax[1].plot(r["tm"], r["w_true"], lw=1.2, label="true yaw rate")
    ax[1].set_ylabel("angular w [rad/s]")
    ax[2].plot(t, T["yaw_gt"], color="k")
    ax[2].set_ylabel("true heading [rad]\n(0 = +x, 1.57 = +y)")
    ax[2].set_xlabel("time [s]")
    for a in ax:
        shade_idle(a, r)
        a.grid(alpha=0.3)
    ax[0].legend(fontsize=8, ncol=2)
    ax[0].set_title(f"{os.path.basename(r['prefix'])}: command vs what the robot actually did", fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("prefix")
    ap.add_argument("--map", help="map yaml to draw under the XY plot")
    ap.add_argument("--compare", help="second run prefix: print both summaries side by side")
    ap.add_argument("--out", help="output dir (default: next to the run)")
    ap.add_argument("--tail", type=float, default=1.0, help="path_logger tail_s used for the run")
    a = ap.parse_args()

    r = analyse(a.prefix, a.tail)
    out_dir = a.out or os.path.dirname(os.path.abspath(r["prefix"]))
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.join(out_dir, os.path.basename(r["prefix"]))

    fig_xy(r, read_map(a.map) if a.map else None, base + "_xy.png")
    fig_errors(r, base + "_errors.png")
    fig_vel(r, base + "_vel.png")

    m, notes = metrics(r)
    lines = [f"== {os.path.basename(r['prefix'])} =="] + [f"{k:38s} {v}" for k, v in m]
    lines += ["", "* true pose assumes the map frame equals the MuJoCo world frame (true once AMCL has converged)"]
    lines += [f"NOTE: {n}" for n in notes]
    if a.compare:
        m2, notes2 = metrics(analyse(a.compare, a.tail))
        lines = ["", f"== compare: {os.path.basename(r['prefix'])}  vs  {os.path.basename(a.compare)} =="]
        d1, d2 = dict(m), dict(m2)
        keys = [k for k, _ in m] + [k for k, _ in m2 if k not in d1]
        lines += [f"{k:38s} {d1.get(k, '-'):48s} | {d2.get(k, '-')}" for k in keys]
        lines += [f"NOTE ({os.path.basename(a.compare)}): {n}" for n in notes2]
    text = "\n".join(lines)
    print(text)
    open(base + "_summary.txt", "w").write(text + "\n")
    print(f"\nwrote {base}_xy.png, _errors.png, _vel.png, _summary.txt")


if __name__ == "__main__":
    main()
