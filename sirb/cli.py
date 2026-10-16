"""Command line entry point: ``python -m sirb {validate,plan,check,margin}``.

Files in, files out. Nothing here talks to an aircraft, a radio or a ground station.
Exit status: 0 = ok, 1 = invalid input or failed check, 2 = the plan verified but FAILED a limit.
"""

from __future__ import annotations

import argparse
import sys

from . import SAFETY_STATEMENT
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

    show = load_show(a.show)
    errs = validate_show(show)
    if errs:
        for e in errs:
            print("error:", e, file=sys.stderr)
        return 1
    cfg = to_stress_config(load(a.gnss), base=StressConfig(dt=0.2, w20=a.w20))
    r = synthesise(show, cfg, target=a.target, trials=a.trials, seed=a.seed)
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


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="sirb", description=SAFETY_STATEMENT)
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
    m.add_argument("--report", help="write the full search record (.json)")
    m.add_argument("--out", help="export bundle of the plan at the chosen scale (.json)")
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
        show, fleet, cert = load_export(a.export)
        print("export hash and recomputed certificate agree")
        return _print_cert(cert)
    except (OSError, ValueError, RuntimeError, KeyError) as exc:
        print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
