#!/usr/bin/env python3
"""
Pre-selection of the discrete left/right choice, with hysteresis — notes 4.2.6.

`homotopy_flips.py` measures the problem: the NLP is smooth and, at the deployed
weights, has a unique minimum, so it never decides which side of an obstacle to
pass.  That decision is delegated to A*, which re-plans from scratch every few
cycles with no memory of what it chose last time, and on a symmetric obstacle it
changes its mind repeatedly.

The course names the remedy.  Notes 4.2.6, on mixed-integer programs:

    "In general, it is advisable to avoid having a large number of integer
     variables in an optimization problem. [...] in optimal control problems one
     can pre-select a switching strategy with continuous tuning parameters,
     obtaining a trade-off between the considered degrees of freedom and the
     complexity of the resulting optimization problem."

"Which side do I pass on" is exactly the integer variable the formulation refuses
to carry.  This module pre-selects it, and `delta` is the continuous tuning
parameter that governs the trade-off.

How it works, at each re-plan:

  1. A* runs normally and produces a candidate route.
  2. Its homotopy signature is compared with the one currently committed.
  3. If they agree, the candidate is accepted and nothing else is computed.
  4. If they disagree, an incumbent route in the COMMITTED class is obtained,
     from two sources in order of preference: the route already being followed,
     re-anchored at the current pose and checked to be still collision-free; or,
     failing that, a fresh A* run behind a temporary barrier on the side the
     challenger chose.  Retaining the route is what hysteresis plainly means and
     is almost always available; the barrier is the fallback, and can legitimately
     fail when the committed side is no longer reachable from here.  Challenger
     and incumbent are then scored against the REAL obstacles:

         J(route) = length + w_clear * integral of max(0, d_ref - clearance)

  5. The challenger is adopted only if it beats the incumbent by more than
     `delta`.  Otherwise the incumbent class is kept.

Two properties this is built to have:

  * Nothing about the NLP changes.  The MPC receives a reference of the same
     shape it always did; J* on any single cycle is untouched.  The layer sits
     entirely upstream, which is also why it is measurable with the existing
     harness.
  * With `--delta 0` the behaviour is not quite the baseline -- ties still
     resolve towards the incumbent -- so the honest comparison is `--off`,
     which restores the unmodified planner exactly.

Usage:
    python3 viz/homotopy_lock.py                        # before/after, all scenarios
    python3 viz/homotopy_lock.py --scenario u_trap --delta 0.5
    python3 viz/homotopy_lock.py --sweep 0 0.25 0.5 1.0 2.0
"""
from __future__ import annotations

import argparse
import contextlib
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common                                            # noqa: E402
import homotopy_flips as hf                              # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")

# Hysteresis margin, in metres of route cost.  A challenger must be at least
# this much cheaper than the committed route before the side is changed.
DELTA_M = 0.5
# Clearance the route scoring would like to keep; shortfalls are penalised.
D_REF_M = 0.45
# Weight of the clearance shortfall against route length, in metres per metre.
W_CLEAR = 6.0
# Length of the temporary barrier used to generate the alternative route.
BARRIER_M = 3.0


# ---------------------------------------------------------------------------
# Route scoring — the A* cost algebra, plus a gap term
# ---------------------------------------------------------------------------
def route_score(path: np.ndarray, obstacles: np.ndarray,
                d_ref: float = D_REF_M, w_clear: float = W_CLEAR) -> float:
    """
    Cost of a candidate route: its length, plus the clearance it gives up.

    The length term is A*'s own g; the clearance term is the "gap width"
    criterion, written as the integral of the shortfall below `d_ref` so that a
    route squeezing past an obstacle pays for it in the same units as a detour.
    """
    p = np.atleast_2d(np.asarray(path, dtype=float))[:, :2]
    if len(p) < 2:
        return float("inf")
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    length = float(seg.sum())
    if obstacles is None or len(obstacles) == 0:
        return length
    d = np.linalg.norm(p[:, None, :] - np.atleast_2d(obstacles)[None, :, :], axis=2)
    short = np.maximum(0.0, d_ref - d.min(axis=1))
    # trapezoid over the arc length, so the penalty does not depend on sampling
    mid = 0.5 * (short[:-1] + short[1:])
    return length + w_clear * float((mid * seg).sum())


def reanchor(path: np.ndarray, pose, min_clearance: float,
             obstacles: np.ndarray) -> np.ndarray | None:
    """
    The previously committed route, re-anchored at the current pose.

    Hysteresis in its plainest form is "keep doing what you were doing", so the
    incumbent candidate should be the route already being followed, truncated to
    the part still ahead of the robot.  Returns None when that route is no longer
    usable -- it has been consumed, or the world has moved into it -- in which
    case there is nothing to hold and the challenger must be accepted.
    """
    if path is None or len(path) < 2:
        return None
    p = np.atleast_2d(np.asarray(path, dtype=float))[:, :2]
    here = np.asarray(pose[:2], dtype=float)
    j = int(np.argmin(np.linalg.norm(p - here, axis=1)))
    tail = p[j:]
    if len(tail) < 2:
        return None                      # the route has been used up
    out = np.vstack([here, tail])
    if obstacles is not None and len(obstacles):
        d = np.linalg.norm(out[:, None, :] - np.atleast_2d(obstacles)[None, :, :],
                           axis=2).min()
        if d < min_clearance:
            return None                  # no longer collision-free: do not hold it
    return out


def barrier_points(pts: np.ndarray, side: int, origin: np.ndarray,
                   axis: np.ndarray, span: float = BARRIER_M,
                   spacing: float = 0.12) -> np.ndarray:
    """
    Virtual points barring one side of a cluster, used only to generate the
    alternative route.  They extend outward from the cluster's extreme point on
    that side, so A* has to come back around the other way.

    These points never reach the MPC and never enter any score: they exist for
    the duration of one A* call.
    """
    normal = np.array([-axis[1], axis[0]])
    y = (pts - origin) @ normal
    q = pts[int(np.argmax(y) if side > 0 else np.argmin(y))]
    n = max(1, int(span / spacing))
    steps = np.arange(1, n + 1)[:, None] * spacing
    return q + steps * (side * normal)[None, :]


# ---------------------------------------------------------------------------
# The selector
# ---------------------------------------------------------------------------
class RouteSelector:
    """
    A stateful stand-in for `common.plan_astar` that keeps the committed
    homotopy class unless a challenger is `delta` cheaper.
    """

    def __init__(self, goal, delta: float = DELTA_M, engage: float = hf.ENGAGE_M,
                 link: float = hf.LINK_M, d_ref: float = D_REF_M,
                 w_clear: float = W_CLEAR, span: float = BARRIER_M):
        self.goal = np.asarray(goal, dtype=float)[:2]
        self.delta = float(delta)
        self.engage, self.link = float(engage), float(link)
        self.d_ref, self.w_clear, self.span = float(d_ref), float(w_clear), float(span)

        self._plain = common.plan_astar        # the unwrapped planner
        self.origin = None                     # mission frame, fixed at first call
        self.axis = None
        self.committed: dict = {}              # cluster -> side
        self.committed_path = None             # the route currently being followed
        self.min_hold_clearance = 0.05         # below this a retained route is dead
        self.stats = {"calls": 0, "conflicts": 0, "held": 0,
                      "switched": 0, "no_alternative": 0,
                      "held_retained": 0, "held_replanned": 0}

    # -- frame ---------------------------------------------------------------
    def _set_frame(self, pose):
        self.origin = np.asarray(pose[:2], dtype=float)
        a = self.goal - self.origin
        n = float(np.linalg.norm(a))
        self.axis = a / n if n > 1e-9 else np.array([1.0, 0.0])

    # -- the call the harness makes -----------------------------------------
    def __call__(self, pose, goal, obstacles, raw):
        base = self._plain(pose, goal, obstacles, raw)
        self.stats["calls"] += 1
        if base is None or len(base) < 2:
            return base
        if self.origin is None:
            self._set_frame(pose)

        obs = np.atleast_2d(np.asarray(obstacles, dtype=float))
        clusters = hf.clusters_of(obs, hf.cluster_obstacles(obs, self.link))
        sig = hf.signature(base, clusters, self.origin, self.axis, self.engage)

        conflicts = [i for i, (s, _) in sig.items()
                     if i in self.committed and s != self.committed[i]]
        if not conflicts:
            self._commit(sig, base)
            return base
        self.stats["conflicts"] += 1

        # Two ways to obtain a route in the committed class, in order of
        # preference.  Retaining the route already being followed is what
        # hysteresis means and is almost always available; re-planning behind a
        # temporary barrier is the fallback, and can legitimately fail when the
        # committed side is no longer reachable from here.
        candidates = []
        kept = reanchor(self.committed_path, pose, self.min_hold_clearance, obs)
        if kept is not None:
            kept_sig = hf.signature(kept, clusters, self.origin, self.axis,
                                    self.engage)
            if all(kept_sig.get(i, (sig[i][0], 0))[0] == self.committed[i]
                   for i in conflicts):
                candidates.append(("retained", kept, kept_sig))

        extra = [barrier_points(clusters[i], sig[i][0], self.origin, self.axis,
                                self.span) for i in conflicts]
        alt = self._plain(pose, goal, np.vstack([obs] + extra), raw)
        if alt is not None and len(alt) >= 2:
            candidates.append(("replanned", alt, hf.signature(
                alt, clusters, self.origin, self.axis, self.engage)))

        if not candidates:
            self.stats["no_alternative"] += 1
            self._commit(sig, base)         # the class cannot be held: accept
            return base

        # score everything against the REAL obstacles; the barrier is not a cost
        j_new = route_score(base, obs, self.d_ref, self.w_clear)
        kind, inc, inc_sig = min(
            candidates, key=lambda c: route_score(c[1], obs, self.d_ref,
                                                  self.w_clear))
        j_old = route_score(inc, obs, self.d_ref, self.w_clear)

        if j_new < j_old - self.delta:
            self.stats["switched"] += 1
            self._commit(sig, base)
            return base

        self.stats["held"] += 1
        self.stats["held_" + kind] += 1
        self._commit(inc_sig, inc)
        return inc

    def _commit(self, sig, path=None):
        for i, (s, _) in sig.items():
            self.committed[i] = s
        if path is not None:
            self.committed_path = np.asarray(path, dtype=float)


@contextlib.contextmanager
def patched(selector):
    """Swap the planner the harness calls, leaving `common.closed_loop` untouched."""
    original = common.plan_astar
    common.plan_astar = selector
    try:
        yield
    finally:
        common.plan_astar = original


# ---------------------------------------------------------------------------
# Measurement
# ---------------------------------------------------------------------------
def measure(name: str, cfg, raw, steps: int, replan_every: int,
            selector: RouteSelector | None, engage: float, link: float) -> dict:
    sc = common.get_scenario(name)
    tracker = common.make_tracker(cfg)

    ctx = patched(selector) if selector is not None else contextlib.nullcontext()
    with ctx:
        hist = common.closed_loop(tracker, sc, steps=steps, raw=raw,
                                  replan_every=replan_every)

    obs = np.asarray(sc.obstacles, dtype=float)
    clusters = hf.clusters_of(obs, hf.cluster_obstacles(obs, link))
    origin = np.asarray(sc.pose[:2], dtype=float)
    axis = sc.goal - origin
    axis = axis / max(float(np.linalg.norm(axis)), 1e-9)

    res = hf.flips_from_history(hist["ref"], clusters, origin, axis, engage)
    res.pop("trace", None)

    pose = np.asarray(hist["pose"], dtype=float)
    res.update({
        "scenario": name,
        "cycles": int(len(pose)),
        "goal_reached": bool(np.linalg.norm(pose[-1, :2] - sc.goal) < 0.35),
        "final_distance_m": round(float(np.linalg.norm(pose[-1, :2] - sc.goal)), 3),
        "min_clearance_m": round(float(common.clearance(pose[:, :2], sc.obstacles)), 3),
        "success_rate": round(float(np.mean(hist["success"])), 3),
        "solve_ms_p95": round(float(np.percentile(hist["solve_ms"], 95)), 1),
        "path_length_m": round(float(np.linalg.norm(np.diff(pose[:, :2], axis=0),
                                                    axis=1).sum()), 3),
    })
    if selector is not None:
        res["selector"] = dict(selector.stats)
    return res


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default=None, choices=sorted(common.SCENARIOS))
    ap.add_argument("--profile", default=common.DEFAULT_PROFILE)
    ap.add_argument("--delta", type=float, default=DELTA_M)
    ap.add_argument("--sweep", type=float, nargs="*", default=None,
                    metavar="DELTA", help="compare several hysteresis margins")
    ap.add_argument("--off", action="store_true",
                    help="baseline only: the unmodified planner")
    ap.add_argument("--steps", type=int, default=250)
    ap.add_argument("--replan-every", type=int, default=5)
    ap.add_argument("--engage", type=float, default=hf.ENGAGE_M)
    ap.add_argument("--link", type=float, default=hf.LINK_M)
    ap.add_argument("--d-ref", type=float, default=D_REF_M)
    ap.add_argument("--w-clear", type=float, default=W_CLEAR)
    ap.add_argument("--set", dest="overrides", action="append", default=[],
                    metavar="KEY=VALUE")
    ap.add_argument("--out", default=os.path.join(OUT, "homotopy_lock.json"))
    args = ap.parse_args()

    cfg, raw = common.load_profile(args.profile, args.overrides)
    names = [args.scenario] if args.scenario else sorted(common.SCENARIOS)
    deltas = args.sweep if args.sweep else [args.delta]

    print(f"profilo {os.path.basename(args.profile)} · N={cfg.N} dt={cfg.dt} "
          f"W_obs={cfg.W_obs_sigmoid:g}")
    print(f"scoring: d_ref={args.d_ref} m · w_clear={args.w_clear} · "
          f"engage={args.engage} m")
    print()

    rows = []
    for n in names:
        base = measure(n, cfg, raw, args.steps, args.replan_every, None,
                       args.engage, args.link)
        rows.append({"scenario": n, "delta": None, **base})
        print(f"  {n:<16} OFF          flips {base['flips']:>2} · "
              f"clearance {base['min_clearance_m']:.3f} · "
              f"path {base['path_length_m']:.2f} m · "
              f"success {100*base['success_rate']:.0f}%")
        if args.off:
            continue
        for d in deltas:
            sel = RouteSelector(common.get_scenario(n).goal, delta=d,
                                engage=args.engage, link=args.link,
                                d_ref=args.d_ref, w_clear=args.w_clear)
            r = measure(n, cfg, raw, args.steps, args.replan_every, sel,
                        args.engage, args.link)
            rows.append({"scenario": n, "delta": d, **r})
            st = r["selector"]
            print(f"  {n:<16} delta={d:<6g} flips {r['flips']:>2} · "
                  f"clearance {r['min_clearance_m']:.3f} · "
                  f"path {r['path_length_m']:.2f} m · "
                  f"success {100*r['success_rate']:.0f}% · "
                  f"held {st['held']}/{st['conflicts']}"
                  + (f" · no alt {st['no_alternative']}"
                     if st["no_alternative"] else ""))
        print()

    print("| scenario | delta | flips | clearance [m] | path [m] | success | held/conflicts |")
    print("|---|---|---|---|---|---|---|")
    for r in rows:
        d = "off" if r["delta"] is None else f"{r['delta']:g}"
        st = r.get("selector")
        hc = "—" if st is None else f"{st['held']}/{st['conflicts']}"
        print(f"| {r['scenario']} | {d} | **{r['flips']}** | "
              f"{r['min_clearance_m']:.3f} | {r['path_length_m']:.2f} | "
              f"{100*r['success_rate']:.0f}% | {hc} |")

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump({"profile": args.profile, "d_ref": args.d_ref,
                   "w_clear": args.w_clear, "engage_m": args.engage,
                   "link_m": args.link, "replan_every": args.replan_every,
                   "runs": rows}, f, indent=2)
    print(f"\nsalvato: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
