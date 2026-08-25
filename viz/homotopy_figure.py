#!/usr/bin/env python3
"""
Before/after figure for the homotopy pre-selection — Section sec:homotopy.

Draws, side by side, every A* reference produced during one mission, coloured by
which side of the obstacle it passes on, with the executed trajectory on top.

  left  : the planner as deployed.  The references alternate colour, which is the
          reference generator changing its mind mid-mission.
  right : with the hysteresis layer.  One colour: the decision is taken once.

The long horizon (--set mpc_N=40) is the informative configuration, because there
the executed trajectory differs too.  At the deployed N=15 the picture shows the
references flipping while the black curve stays identical -- which is the point of
the paragraph "the consequence is gated by the horizon", and worth generating as
its own figure.

Usage:
    python3 viz/homotopy_figure.py                          # u_trap at N=40
    python3 viz/homotopy_figure.py --scenario corridor
    python3 viz/homotopy_figure.py --set mpc_N=15 --tag deployed
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import matplotlib
if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt                              # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common                                                # noqa: E402
import homotopy_flips as hf                                  # noqa: E402
import homotopy_lock as hl                                   # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")

C_LEFT = "#1b6ca8"      # reference passing on one side
C_RIGHT = "#c1440e"     # ... and on the other
C_TRAJ = "#111111"
C_OBS = "#444444"


def run(name, cfg, raw, steps, replan_every, selector, engage, link):
    """One mission; returns the history, the distinct references and their sides."""
    sc = common.get_scenario(name)
    tracker = common.make_tracker(cfg)
    import contextlib
    ctx = hl.patched(selector) if selector is not None else contextlib.nullcontext()
    with ctx:
        hist = common.closed_loop(tracker, sc, steps=steps, raw=raw,
                                  replan_every=replan_every)

    obs = np.asarray(sc.obstacles, dtype=float)
    clusters = hf.clusters_of(obs, hf.cluster_obstacles(obs, link))
    origin = np.asarray(sc.pose[:2], dtype=float)
    axis = sc.goal - origin
    axis = axis / max(float(np.linalg.norm(axis)), 1e-9)

    refs, sides, seen = [], [], None
    for r in hist["ref"]:
        if r is None:
            continue
        if seen is not None and r.shape == seen.shape and np.allclose(r, seen):
            continue
        seen = np.asarray(r, dtype=float)
        sig = hf.signature(seen, clusters, origin, axis, engage)
        refs.append(seen[:, :2])
        # one colour per reference: the side of the first engaged cluster
        sides.append(sig[min(sig)][0] if sig else 0)

    stats = hf.flips_from_history(hist["ref"], clusters, origin, axis, engage)
    pose = np.asarray(hist["pose"], dtype=float)
    stats.update({
        "path_m": float(np.linalg.norm(np.diff(pose[:, :2], axis=0), axis=1).sum()),
        "clearance_m": float(common.clearance(pose[:, :2], sc.obstacles)),
        "success": float(np.mean(hist["success"])),
    })
    return sc, pose, refs, sides, stats


def panel(ax, sc, pose, refs, sides, stats, title):
    ax.scatter(sc.obstacles[:, 0], sc.obstacles[:, 1], s=7, c=C_OBS,
               marker="s", zorder=2, label="obstacles")

    drawn = set()
    for r, s in zip(refs, sides):
        c = C_LEFT if s > 0 else (C_RIGHT if s < 0 else "#999999")
        lab = None
        key = "L" if s > 0 else ("R" if s < 0 else "-")
        if key not in drawn:
            lab = {"L": "A* reference, one side", "R": "A* reference, other side",
                   "-": "A* reference, obstacle not engaged"}[key]
            drawn.add(key)
        ax.plot(r[:, 0], r[:, 1], c=c, lw=1.1, alpha=0.75, zorder=3, label=lab)

    ax.plot(pose[:, 0], pose[:, 1], c=C_TRAJ, lw=2.4, zorder=4,
            label="executed trajectory")
    ax.plot(*sc.pose[:2], "o", c=C_TRAJ, ms=7, zorder=5)
    ax.plot(*sc.goal, "*", c="#1a7f37", ms=16, zorder=5, label="goal")

    ax.set_title(title, fontsize=11, pad=8)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25, lw=0.5)
    ax.set_xlabel("x [m]")
    txt = (f"flips {stats['flips']}   ·   path {stats['path_m']:.2f} m   ·   "
           f"clearance {stats['clearance_m']:.3f} m   ·   "
           f"solved {100*stats['success']:.0f}%")
    ax.text(0.5, -0.17, txt, transform=ax.transAxes, ha="center", fontsize=9.5,
            family="monospace")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="u_trap", choices=sorted(common.SCENARIOS))
    ap.add_argument("--profile", default=common.DEFAULT_PROFILE)
    ap.add_argument("--delta", type=float, default=hl.DELTA_M)
    ap.add_argument("--steps", type=int, default=250)
    ap.add_argument("--replan-every", type=int, default=5)
    ap.add_argument("--engage", type=float, default=hf.ENGAGE_M)
    ap.add_argument("--link", type=float, default=hf.LINK_M)
    ap.add_argument("--set", dest="overrides", action="append",
                    default=["mpc_N=40"], metavar="KEY=VALUE")
    ap.add_argument("--tag", default=None, help="suffix for the output file name")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    cfg, raw = common.load_profile(args.profile, args.overrides)
    print(f"scenario {args.scenario} · N={cfg.N} dt={cfg.dt} W_obs={cfg.W_obs_sigmoid:g} "
          f"· delta={args.delta}")

    a = run(args.scenario, cfg, raw, args.steps, args.replan_every, None,
            args.engage, args.link)
    sel = hl.RouteSelector(common.get_scenario(args.scenario).goal, delta=args.delta,
                           engage=args.engage, link=args.link)
    b = run(args.scenario, cfg, raw, args.steps, args.replan_every, sel,
            args.engage, args.link)

    for label, r in (("deployed planner", a), ("with pre-selection", b)):
        print(f"  {label:<20} flips {r[4]['flips']} · path {r[4]['path_m']:.2f} m · "
              f"clearance {r[4]['clearance_m']:.3f} m")

    fig, axes = plt.subplots(1, 2, figsize=(11.5, 5.2), sharex=True, sharey=True)
    panel(axes[0], *a, f"(a) as deployed — A* re-plans with no memory")
    panel(axes[1], *b, f"(b) with pre-selection, $\\delta={args.delta:g}$ m")
    axes[0].set_ylabel("y [m]")
    h, l = axes[0].get_legend_handles_labels()
    h2, l2 = axes[1].get_legend_handles_labels()
    for hh, ll in zip(h2, l2):                 # union, keeping first occurrence
        if ll not in l:
            h.append(hh); l.append(ll)
    fig.legend(h, l, loc="lower center", ncol=len(l), fontsize=9,
               frameon=False, bbox_to_anchor=(0.5, -0.02))
    fig.suptitle(f"Homotopy class of the reference over one mission — "
                 f"{args.scenario}, $N={cfg.N}$, $\\Delta t={cfg.dt:g}$ s",
                 fontsize=12.5)
    fig.tight_layout(rect=(0, 0.06, 1, 0.97))

    tag = args.tag or f"N{cfg.N}"
    base = os.path.join(OUT, f"homotopy_{args.scenario}_{tag}")
    os.makedirs(OUT, exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(f"{base}.{ext}", dpi=160, bbox_inches="tight")
        print(f"salvato: {base}.{ext}")
    if not args.no_show and os.environ.get("DISPLAY"):
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
