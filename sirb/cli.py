"""Command line entry point: ``python -m sirb {validate,plan,check,margin,stress,verify-external}``.

Files in, files out. Nothing here talks to an aircraft, a radio or a ground station.
Exit status: 0 = ok, 1 = invalid input or failed check, 2 = the plan verified but FAILED a limit.
"""

from __future__ import annotations

import argparse
import sys

from . import SAFETY_STATEMENT, __version__
from .export import build_export, load_export, save_export
from .spec import load_show, validate_show

PLANNERS = ("baseline", "waves", "prioritized")


def _plan(show: dict, planner: str):
    if planner == "baseline":
        from .planner.baseline import plan_baseline as fn
    elif planner == "waves":
        from .planner.waves import plan_waves as fn
    else:
        from .planner.prioritized import plan_prioritized as fn
    return fn(show)


def _print_cert(cert: dict) -> int:
    res = cert["results"]
    bound = " (lower bound: no pair came within the search radius)" \
        if res.get("min_separation_is_lower_bound") else ""
    print(f"verdict: {cert['verdict']} (against the limits declared in the show file only)   "
          "[simulation-validated only; not an approval]")
    print(f"drones: {cert['n_drones']}   fleet sha256: {cert['fleet_sha256']}")
    print(f"minimum centre separation: {res['min_separation']:.4f} m{bound}; "
          f"declared threshold {cert['limits']['min_separation_m']} m")
    if res.get("violation_counts"):
        print("violations:", res["violation_counts"])
    return 0 if cert["verdict"] == "PASS" else 2


def _margin(a) -> int:
    import json

    from .disturbance import StressConfig
    from .gnss_params import load, to_stress_config
    from .margin import scale_show, synthesise
    from .planner.baseline import plan_baseline

    if not 0.0 < a.target < 1.0:
        print("error: --target must lie strictly between 0 and 1", file=sys.stderr)
        return 1
    if a.trials < 1 or a.tol <= 0 or a.w20 < 0:
        print("error: --trials must be at least 1, --tol positive and --w20 not negative",
              file=sys.stderr)
        return 1
    show = load_show(a.show)
    errs = validate_show(show)
    if errs:
        for e in errs:
            print("error:", e, file=sys.stderr)
        return 1
    cfg = to_stress_config(load(a.gnss), base=StressConfig(dt=0.2, w20=a.w20))
    r = synthesise(show, cfg, target=a.target, trials=a.trials, seed=a.seed, tol=a.tol)
    print(r["model_dependence"])
    if a.report:
        with open(a.report, "w", encoding="utf-8") as f:
            json.dump(r, f, indent=1)
    if r["factor"] is None:
        print(f"no qualifying scale: {r['status']}")
        return 2
    print(f"scale factor {r['factor']:.4f}: nominal minimum separation "
          f"{r['required_nominal_min_separation_m']:.3f} m (d_min {r['d_min']} m) gives a Wilson "
          f"upper bound at most {a.target} over {a.trials} simulated trials")
    if a.out:
        scaled = scale_show(show, r["factor"])
        fleet, _ = plan_baseline(scaled)
        save_export(a.out, build_export(scaled, fleet))
        print(f"wrote {a.out}")
    return 0


def _verify_external(a) -> int:
    from .importers import fleet_from_csv
    from .verify import pairwise_min_separation

    if not a.d_min > 0:
        print("error: --d-min must be positive", file=sys.stderr)
        return 1
    fleet = fleet_from_csv(a.csv)
    if a.d_min_vertical is not None:
        from .clearance import ellipsoidal_min_separation
        if a.d_min_vertical < a.d_min:
            print("error: --d-min-vertical must be at least --d-min", file=sys.stderr)
            return 1
        d, i, j, t, lb, _ = ellipsoidal_min_separation(fleet, a.d_min, a.d_min_vertical,
                                                       a.search_radius)
        print(f"ellipsoidal check: {a.d_min} m horizontal, {a.d_min_vertical} m vertical")
    else:
        d, i, j, t, lb, _ = pairwise_min_separation(fleet, a.search_radius)
    ok = d >= a.d_min
    bound = " (lower bound: no pair came within the search radius)" if lb else ""
    print(f"drones: {len(fleet.trajectories)}   fleet sha256: {fleet.content_hash()}")
    print(f"minimum centre separation: {d:.4f} m{bound} between drones {i} and {j} at t = {t:.3f} s")
    print(f"verdict: {'PASS' if ok else 'FAIL'} against {a.d_min} m. Samples joined by straight "
          "lines; separation only; simulation-validated only, not an approval.")
    return 0 if ok else 2


def _stress(a) -> int:
    import json

    from .disturbance import StressConfig
    from .gnss_params import load, to_stress_config
    from .stress import monte_carlo, sample_reference

    if a.trials < 1 or a.workers < 1 or a.w20 < 0:
        print("error: --trials and --workers must be at least 1 and --w20 not negative",
              file=sys.stderr)
        return 1
    show, fleet, _ = load_export(a.export)
    if not 0.0 <= a.gnss_common_fraction <= 1.0:
        print("error: --gnss-common-fraction must lie between 0 and 1", file=sys.stderr)
        return 1
    cfg = to_stress_config(load(a.gnss), base=StressConfig(
        dt=0.2, w20=a.w20, gnss_common_fraction=a.gnss_common_fraction))
    d_min = show["limits"]["min_separation_m"]
    r = monte_carlo(sample_reference(fleet, cfg.dt), cfg, d_min, a.trials, a.seed,
                    workers=a.workers)
    lo, hi = r["p_violation_ci95"]
    print(f"SIMULATED under assumed disturbance models; not flight evidence. d_min {d_min} m, "
          f"{a.trials} trials, seed {a.seed}")
    print(f"P(separation violation) = {r['p_violation']:.3f}  (Wilson 95%: {lo:.3f} to {hi:.3f})")
    q = r["min_separation_quantiles"]
    print(f"minimum separation quantiles (m): min {q['0.0']:.3f}, 5% {q['0.05']:.3f}, "
          f"median {q['0.5']:.3f}")
    if a.report:
        with open(a.report, "w", encoding="utf-8") as f:
            json.dump(r, f, indent=1)
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="sirb", description=SAFETY_STATEMENT)
    ap.add_argument("--version", action="version", version=f"sirb {__version__}")
    sub = ap.add_subparsers(dest="cmd", required=True)
    v = sub.add_parser("validate", help="validate a show specification file")
    v.add_argument("show")
    p = sub.add_parser("plan", help="plan a show, verify it, write an export bundle")
    p.add_argument("show")
    p.add_argument("--planner", choices=PLANNERS, default="baseline")
    p.add_argument("--out", required=True, help="export bundle path (.json)")
    c = sub.add_parser("check", help="re-check an export bundle (hash and recomputed certificate)")
    c.add_argument("export")
    m = sub.add_parser("margin", help="planning aid: nominal separation needed for a simulated "
                       "risk target, and a plan scaled to it (model dependent)")
    m.add_argument("show")
    m.add_argument("--gnss", required=True, help="GNSS parameter file (docs/GNSS_PARAMS.md)")
    m.add_argument("--w20", type=float, default=0.0, help="mean wind at 20 ft, m/s (assumed)")
    m.add_argument("--target", type=float, default=0.05, help="Wilson upper bound target")
    m.add_argument("--trials", type=int, default=200)
    m.add_argument("--seed", type=int, default=20261016)
    m.add_argument("--tol", type=float, default=0.02, help="relative tolerance of the scale search")
    m.add_argument("--report", help="write the full search record (.json)")
    m.add_argument("--out", help="export bundle of the plan at the chosen scale (.json)")
    x = sub.add_parser("verify-external", help="exactly verify separation of a sampled "
                       "trajectory CSV (drone,t,x,y,z)")
    x.add_argument("csv")
    x.add_argument("--d-min", type=float, required=True, help="required centre separation, m")
    x.add_argument("--d-min-vertical", type=float, default=None,
                   help="required vertical separation, m (ellipsoid check; at least --d-min)")
    x.add_argument("--search-radius", type=float, default=50.0, help="pair search radius, m")
    t = sub.add_parser("stress", help="Monte Carlo stress test of an export bundle (simulated)")
    t.add_argument("export")
    t.add_argument("--gnss", required=True, help="GNSS parameter file (docs/GNSS_PARAMS.md)")
    t.add_argument("--w20", type=float, default=0.0, help="mean wind at 20 ft, m/s (assumed)")
    t.add_argument("--trials", type=int, default=200)
    t.add_argument("--seed", type=int, default=20261016)
    t.add_argument("--gnss-common-fraction", type=float, default=0.0,
                   help="share of the GNSS bias variance common to all degraded drones, 0 to 1 "
                        "(default 0: errors independent per drone, the pessimistic reading)")
    t.add_argument("--workers", type=int, default=1)
    t.add_argument("--report", help="write the full result (.json)")
    a = ap.parse_args(argv)

    try:
        if a.cmd == "validate":
            errs = validate_show(load_show(a.show))
            for e in errs:
                print("error:", e, file=sys.stderr)
            print("show is valid" if not errs else f"{len(errs)} error(s)")
            return 0 if not errs else 1
        if a.cmd == "plan":
            show = load_show(a.show)
            errs = validate_show(show)
            if errs:
                for e in errs:
                    print("error:", e, file=sys.stderr)
                return 1
            fleet, info = _plan(show, a.planner)
            bundle = build_export(show, fleet)
            save_export(a.out, bundle)
            print(f"planned with {a.planner} in {info['plan_runtime_s']:.1f} s; wrote {a.out}")
            return _print_cert(bundle["certificate"])
        if a.cmd == "margin":
            return _margin(a)
        if a.cmd == "verify-external":
            return _verify_external(a)
        if a.cmd == "stress":
            return _stress(a)
        show, fleet, cert = load_export(a.export)
        print("export hash and recomputed certificate agree")
        return _print_cert(cert)
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
