"""Deterministic 1000 drone synthetic dataset for the viewer benchmark (same .bin format as the viewer data).

Show: launch grid -> sphere -> launch grid (landing), built with sirb.spec.new_show, planned with the
baseline planner, checked with the exact verifier, and written with the same writers as
scripts/build_viewer_data.py. The geofence is enlarged because 1000 drones do not fit the demo fence.
The verifier verdict is recorded in index.json (and printed); if it is not PASS the dataset is only a
rendering load and is labelled as such.

    python scripts/build_viewer_bench_data.py [outdir]     # default: results/bench_data (not committed)
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

from build_viewer_data import DT, SCENARIOS, SEED, round_floats, write_bin

from sirb import SAFETY_STATEMENT
from sirb.certificate import make_certificate, verify_certificate
from sirb.disturbance import StressConfig
from sirb.planner.baseline import plan_baseline
from sirb.spec import new_show
from sirb.stress import realize, sample_reference
from sirb.verify import verify

LAUNCH = {"type": "launch_grid", "spacing_m": 3.0, "center": [0.0, -85.0, 0.0]}
N = 1000


def bench_show() -> dict:
    return new_show("Synthetic bench (1000 drones)", N, [
        {"name": "launch", "shape": LAUNCH, "hold_s": 2},
        {"name": "orb", "shape": {"type": "sphere", "spacing_m": 3.5, "center": [0, 0, 60]}, "hold_s": 4},
        {"name": "land", "shape": LAUNCH, "hold_s": 2},
    ], seed=55, limits={"geofence": {"x": [-300.0, 300.0], "y": [-300.0, 200.0]},
                        "crowd": {"y": -400.0, "min_distance_m": 50.0}})


def main(outdir: str = "results/bench_data") -> None:
    out = Path(outdir)
    d = out / "bench_1000"
    d.mkdir(parents=True, exist_ok=True)
    show = bench_show()
    t0 = time.time()
    fleet, _ = plan_baseline(show)
    t_plan = time.time() - t0
    rep = verify(fleet, show["limits"])
    cert = make_certificate(rep, fleet, show["limits"])
    ref = sample_reference(fleet, DT)
    K, n, _ = ref.shape
    (d / "certificate.json").write_text(json.dumps(cert, indent=1))
    sc = [{"id": "nominal", "label_en": "Nominal (synthetic)", "label_ar": "اسمي (اصطناعي)",
           **write_bin(d / "nominal.bin", ref)}]
    spec = SCENARIOS["wind_gnss"]
    X, R, eps, events, ginfo, bal = realize(ref, StressConfig(dt=DT).with_(**spec["cfg"]), SEED)
    info = write_bin(d / "wind_gnss.bin", X)
    info.update(id="wind_gnss", label_en=spec["label_en"], label_ar=spec["label_ar"], events=[], ballistic=[],
                max_tracking_error_m=float(abs(eps).max()), config=spec["cfg"])
    sc.append(info)
    index = {"safety_statement": SAFETY_STATEMENT, "dt": DT, "seed": SEED, "pos_scale": 0.01, "synthetic": True,
             "shows": [{"id": "bench_1000", "name": show["name"], "n": int(n), "frames": int(K),
                        "duration_s": float(fleet.t_end), "d_min_m": show["limits"]["min_separation_m"],
                        "geofence": show["limits"]["geofence"], "crowd": show["limits"]["crowd"],
                        "fleet_sha256": fleet.content_hash(), "verdict": cert["verdict"],
                        "min_separation_m": rep.min_separation,
                        "recomputation_ok": bool(verify_certificate(cert, fleet)),
                        "certificate": "bench_1000/certificate.json", "scenarios": sc}]}
    (out / "index.json").write_text(json.dumps(round_floats(index), indent=1))
    print(f"bench_1000: n={n} frames={K} verdict={cert['verdict']} min_sep={rep.min_separation:.3f} "
          f"d_min={show['limits']['min_separation_m']} plan_s={t_plan:.1f} wrote {out}")


if __name__ == "__main__":
    main(*sys.argv[1:2])
