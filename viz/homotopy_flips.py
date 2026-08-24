#!/usr/bin/env python3
"""
Homotopy-class instability of the A* reference — course notes 4.2.6, 4.3.3, 4.4.6.

The NLP never carries the discrete decision "do I pass this obstacle on the left
or on the right".  Notes 4.2.6 recommends exactly that: avoid integer variables,
pre-select the switching strategy, keep the continuous program smooth.  Here the
pre-selection is delegated to A*, which re-plans every `replan_every` cycles on a
freshly inflated grid.

Nothing constrains A* to keep its previous choice.  When it changes side, the
reference handed to the MPC jumps to a different homotopy class, and the tracked
trajectory is discontinuous even though the MPC itself converged cleanly on both
sides.  This script measures how often that happens.

The measurement is a *signature*, not a heuristic.  Obstacle points are grouped
into connected clusters; for every cluster the reference passes near, we record
which side it passes on, read as a lateral offset at the cluster's longitudinal
station in the fixed start-to-goal frame (see `signature` for why the two more
obvious definitions are confounded by the rolling horizon).

A flip is any cluster that is engaged by two consecutive references and whose
sign differs between them.  Clusters engaged by only one of the two are ignored:
entering or leaving the planning window is not a change of mind.

KNOWN LIMITATION -- extended obstacles.  A cluster is represented by its
centroid, which is a good landmark for a compact obstacle (a pillar, a U-shaped
trap) and a poor one for a wall running parallel to the direction of travel: the
path is alongside it for its whole length, so the "station of the centroid" is
arbitrary and the recorded offset can exceed the free width of the corridor.
Trust the count on compact clusters; on `corridor`, whose two long walls are of
this kind, treat it as not yet validated.  The fix is to landmark each cluster by
the point nearest the reference instead of by its centroid.

Usage:
    python3 viz/homotopy_flips.py                          # all four scenarios
    python3 viz/homotopy_flips.py --scenario u_trap
    python3 viz/homotopy_flips.py --engage 2.0 --link 0.35
    python3 viz/homotopy_flips.py --profile src/a_star_mpc_planner/config/planner_params.yaml
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common                                        # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")

# Obstacle points closer than this are one obstacle.  The scenario walls are
# sampled at 0.12 m, so 0.30 links a wall without merging distinct objects.
LINK_M = 0.30
# A cluster counts as "engaged" by a reference when the reference passes within
# this distance of its centroid.  Beyond it the side is not a decision.
ENGAGE_M = 1.5


def cluster_obstacles(obs: np.ndarray, link: float = LINK_M) -> np.ndarray:
    """Label obstacle points by connected component of the radius-`link` graph."""
    obs = np.asarray(obs, dtype=float)
    n = len(obs)
    if n == 0:
        return np.zeros(0, dtype=int)
    pairs = cKDTree(obs).query_pairs(link, output_type="ndarray")
    if len(pairs) == 0:
        return np.arange(n)
    g = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(n, n))
    _, labels = connected_components(g, directed=False)
    return labels


def centroids_of(obs: np.ndarray, labels: np.ndarray) -> np.ndarray:
    return np.array([obs[labels == k].mean(axis=0) for k in np.unique(labels)])


def winding(ref: np.ndarray, c: np.ndarray) -> float:
    """
    Total signed angle swept by the vector (path point - centroid) along `ref`.

    This is the homotopy invariant of an open path in the plane punctured at `c`.
    Integrating the whole path makes it insensitive to local curvature, unlike
    the sign of the tangent-normal cross product at the closest approach, which
    flips whenever the path merely turns.
    """
    v = np.asarray(ref, dtype=float)[:, :2] - c
    a = np.arctan2(v[:, 1], v[:, 0])
    d = np.diff(a)
    d = (d + np.pi) % (2.0 * np.pi) - np.pi      # unwrap each increment
    return float(d.sum())


# Below this swept angle the reference has not really gone past the obstacle,
# so "which side" is not yet a decision it has taken.
MIN_SWEEP_RAD = 0.5


def signature(ref: np.ndarray, centroids: np.ndarray,
              origin: np.ndarray, axis: np.ndarray,
              engage: float = ENGAGE_M) -> dict[int, tuple[int, float]]:
    """
    Which side of each engaged cluster this reference passes on.

    The side is read in a frame FIXED for the whole mission: the mission axis
    from the start pose to the goal.  For each cluster we take the longitudinal
    station of its centroid, find where the reference crosses that station, and
    compare the two lateral offsets.  side = +1 when the reference passes to the
    left of the obstacle, -1 to the right.

    Why not something simpler.  Two natural definitions are both confounded by
    the rolling horizon, and were measured to be so on these scenarios:

      * the sign of cross(tangent, centroid - closest point) flips whenever the
        path merely curves, without changing side;
      * the sign of the winding angle around the centroid flips as the robot
        advances PAST the centroid, so it encodes longitudinal progress rather
        than a decision -- on `corridor` it made all three clusters flip on the
        same cycle, which is the signature of the frame moving, not of A*
        changing its mind.

    Reading the offset at a fixed station removes both: the value only changes
    when A* actually re-routes.

    A cluster is engaged when the reference spans its station and passes within
    `engage` of the centroid.
    """
    ref = np.atleast_2d(np.asarray(ref, dtype=float))[:, :2]
    if len(ref) < 2:
        return {}
    normal = np.array([-axis[1], axis[0]])

    s_ref = (ref - origin) @ axis          # longitudinal station of each sample
    y_ref = (ref - origin) @ normal        # lateral offset of each sample

    sig: dict[int, tuple[int, float]] = {}
    for i, c in enumerate(centroids):
        if np.linalg.norm(ref - c, axis=1).min() > engage:
            continue
        s_c = float((c - origin) @ axis)
        y_c = float((c - origin) @ normal)

        # the reference must actually cross the obstacle's station
        k = np.nonzero((s_ref[:-1] - s_c) * (s_ref[1:] - s_c) <= 0.0)[0]
        if k.size == 0:
            continue
        j = int(k[-1])                      # the last crossing, nearest the goal
        ds = s_ref[j + 1] - s_ref[j]
        w = 0.0 if abs(ds) < 1e-12 else (s_c - s_ref[j]) / ds
        y_cross = float(y_ref[j] + w * (y_ref[j + 1] - y_ref[j]))

        offset = y_cross - y_c
        if abs(offset) < 1e-6:
            continue                        # exactly through the centroid
        sig[i] = (1 if offset > 0 else -1, abs(offset))
    return sig


def legacy_side(ref) -> int:
    """The detector currently in cost_field.py:364, kept for comparison."""
    if ref is None:
        return 0
    r = np.atleast_2d(np.asarray(ref, dtype=float))
    return 1 if r[:, 1].max() > abs(r[:, 1].min()) else -1


def flips_from_history(refs, centroids, origin, axis, engage=ENGAGE_M) -> dict:
    """Walk the per-cycle reference history and count homotopy-class changes."""
    prev_sig, prev_cycle = None, None
    flips, events, sig_trace = 0, [], []

    seen = None
    for k, ref in enumerate(refs):
        if ref is None:
            continue
        # A* only re-plans periodically; the same array is repeated in between.
        if seen is not None and ref.shape == seen.shape and np.allclose(ref, seen):
            continue
        seen = np.asarray(ref, dtype=float)

        sig = signature(seen, centroids, origin, axis, engage)
        sig_trace.append({"cycle": k,
                          "sides": {str(i): s for i, (s, _) in sig.items()},
                          "offset": {str(i): round(c, 3) for i, (_, c) in sig.items()}})

        if prev_sig is not None:
            shared = set(sig) & set(prev_sig)
            changed = [i for i in sorted(shared) if sig[i][0] != prev_sig[i][0]]
            if changed:
                flips += 1
                events.append({"from_cycle": prev_cycle, "to_cycle": k,
                               "clusters": changed})
        prev_sig, prev_cycle = sig, k

    return {"flips": flips, "events": events, "replans": len(sig_trace),
            "trace": sig_trace}


def run_scenario(name: str, cfg, raw, steps: int, replan_every: int,
                 engage: float, link: float) -> dict:
    sc = common.get_scenario(name)
    tracker = common.make_tracker(cfg)
    hist = common.closed_loop(tracker, sc, steps=steps, raw=raw,
                              replan_every=replan_every)

    labels = cluster_obstacles(sc.obstacles, link)
    cent = centroids_of(sc.obstacles, labels)

    origin = np.asarray(sc.pose[:2], dtype=float)
    axis = np.asarray(sc.goal, dtype=float) - origin
    axis = axis / max(float(np.linalg.norm(axis)), 1e-9)
    res = flips_from_history(hist["ref"], cent, origin, axis, engage)

    # the old heuristic, on the same history, for the record
    sides = np.array([legacy_side(r) for r in hist["ref"]])
    nz = sides[sides != 0]
    res["legacy_flips"] = int((np.diff(nz) != 0).sum()) if len(nz) > 1 else 0

    pose = np.asarray(hist["pose"], dtype=float)
    res.update({
        "scenario": name,
        "clusters": int(len(cent)),
        "cycles": int(len(pose)),
        "goal_reached": bool(np.linalg.norm(pose[-1, :2] - sc.goal) < 0.35),
        "final_distance_m": round(float(np.linalg.norm(pose[-1, :2] - sc.goal)), 3),
        "min_clearance_m": round(float(common.clearance(pose[:, :2], sc.obstacles)), 3),
        "success_rate": round(float(np.mean(hist["success"])), 3),
        "solve_ms_p95": round(float(np.percentile(hist["solve_ms"], 95)), 1),
    })
    return res


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default=None, choices=sorted(common.SCENARIOS),
                    help="default: every scenario")
    ap.add_argument("--profile", default=common.DEFAULT_PROFILE)
    ap.add_argument("--steps", type=int, default=250)
    ap.add_argument("--replan-every", type=int, default=5)
    ap.add_argument("--engage", type=float, default=ENGAGE_M,
                    help="distance within which a cluster counts as engaged [m]")
    ap.add_argument("--link", type=float, default=LINK_M,
                    help="distance below which two obstacle points are one object [m]")
    ap.add_argument("--set", dest="overrides", action="append", default=[],
                    metavar="KEY=VALUE")
    ap.add_argument("--out", default=os.path.join(OUT, "homotopy_flips.json"))
    args = ap.parse_args()

    cfg, raw = common.load_profile(args.profile, args.overrides)
    names = [args.scenario] if args.scenario else sorted(common.SCENARIOS)

    print(f"profilo {os.path.basename(args.profile)} · N={cfg.N} dt={cfg.dt} "
          f"W_obs={cfg.W_obs_sigmoid:g} · engage={args.engage} m link={args.link} m")
    print()

    rows = []
    for n in names:
        r = run_scenario(n, cfg, raw, args.steps, args.replan_every,
                         args.engage, args.link)
        rows.append(r)
        print(f"  {n:<16} clusters {r['clusters']:>2} · replans {r['replans']:>3} · "
              f"FLIPS {r['flips']:>2} (legacy {r['legacy_flips']:>2}) · "
              f"clearance {r['min_clearance_m']:.3f} m · "
              f"success {100*r['success_rate']:.0f}%")
        for e in r["events"]:
            print(f"      flip at cycle {e['to_cycle']:>3} "
                  f"(was {e['from_cycle']}) · clusters {e['clusters']}")

    print()
    print("| scenario | clusters | replans | flips | legacy | clearance [m] | success |")
    print("|---|---|---|---|---|---|---|")
    for r in rows:
        print(f"| {r['scenario']} | {r['clusters']} | {r['replans']} | "
              f"**{r['flips']}** | {r['legacy_flips']} | "
              f"{r['min_clearance_m']:.3f} | {100*r['success_rate']:.0f}% |")

    payload = {"profile": args.profile,
               "engage_m": args.engage, "link_m": args.link,
               "replan_every": args.replan_every,
               "scenarios": rows}
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"\nsalvato: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
