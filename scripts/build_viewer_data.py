"""Build the static data bundle consumed by the web viewer (viewer/data/).

For each demo show: plan (baseline planner), verify, certificate, then the nominal motion and a few
fixed-seed stressed realisations. Positions are stored as int16 centimetres (little-endian),
nearest-neighbour distance per drone per frame as uint16 centimetres. Everything is deterministic.

    python scripts/build_viewer_data.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from sirb import SAFETY_STATEMENT
from sirb.certificate import make_certificate, verify_certificate
from sirb.disturbance import StressConfig
from sirb.planner.baseline import plan_baseline
from sirb.spec import load_show
from sirb.stress import realize, sample_reference
from sirb.verify import verify

OUT = Path("viewer/data")
DT = 0.2
SEED = 20261011
SHOWS = ["sirb_text_100", "torus_300", "wall_500"]

SCENARIOS = {
    "wind_gnss": dict(label_en="Wind 6 m/s + degraded GNSS", label_ar="رياح ٦ م/ث + تدهور GNSS",
                      cfg=dict(w20=6.0, gnss_deg_fraction=0.25, gnss_bias_sigma=0.5)),
    "motor_failure": dict(label_en="2 motor failures (ballistic fall), wind 3 m/s",
                          label_ar="عطل محرّكين (سقوط حر)، رياح ٣ م/ث",
                          cfg=dict(w20=3.0, motor_failures=2)),
    "comms_hold": dict(label_en="5% comms loss, hold-then-descend",
                       label_ar="فقدان اتصال ٥٪، تثبيت ثم هبوط",
                       cfg=dict(w20=3.0, comms_loss_fraction=0.05,
                                behavior={"comms_loss": "hold", "battery_low": "descend",
                                          "gnss_loss": "continue", "motor_failure": "ballistic"})),
}


def round_floats(x, nd: int = 6):
    """Round every float in a JSON-like structure. index.json is diffed in CI; rounding keeps
    that check robust to last-bit differences between BLAS/LAPACK builds."""
    if isinstance(x, float):
        return round(x, nd)
    if isinstance(x, dict):
        return {k: round_floats(v, nd) for k, v in x.items()}
    if isinstance(x, list):
        return [round_floats(v, nd) for v in x]
    return x


def nn_distance(X: np.ndarray) -> np.ndarray:
    K, n, _ = X.shape
    out = np.empty((K, n), dtype=np.float64)
    for k in range(K):
        d, _ = cKDTree(X[k]).query(X[k], k=2)
        out[k] = d[:, 1]
    return out


def write_bin(path: Path, X: np.ndarray) -> dict:
    q = np.clip(np.round(X * 100.0), -32767, 32767).astype("<i2")
    q.tofile(path)
    nn = np.clip(np.round(nn_distance(X) * 100.0), 0, 65535).astype("<u2")
    nn.tofile(path.with_suffix(".nn.bin"))
    return {"pos": path.name, "nn": path.with_suffix(".nn.bin").name,
            "min_nn_m": float(nn.min() / 100.0)}


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    index = {"safety_statement": SAFETY_STATEMENT, "dt": DT, "seed": SEED, "pos_scale": 0.01,
             "shows": []}
    for name in SHOWS:
        show = load_show(f"shows/{name}.json")
        fleet, _ = plan_baseline(show)
        rep = verify(fleet, show["limits"])
        cert = make_certificate(rep, fleet, show["limits"])
        independent_ok = bool(verify_certificate(cert, fleet))
        ref = sample_reference(fleet, DT)
        K, n, _ = ref.shape
        d = OUT / name
        d.mkdir(exist_ok=True)
        (d / "certificate.json").write_text(json.dumps(cert, indent=1))
        sc = [{"id": "nominal", "label_en": "Nominal (verified plan)",
               "label_ar": "الخطة الاسمية (متحقَّق منها)", **write_bin(d / "nominal.bin", ref)}]
        for sid, spec in SCENARIOS.items():
            cfg = StressConfig(dt=DT).with_(**spec["cfg"])
            X, R, eps, events, ginfo, bal = realize(ref, cfg, SEED + len(sid))
            info = write_bin(d / f"{sid}.bin", X)
            info.update(id=sid, label_en=spec["label_en"], label_ar=spec["label_ar"],
                        events=[{"drone": int(e["drone"]), "kind": e["kind"],
                                 "t": round(e["k"] * DT, 2)} for e in events],
                        ballistic=[int(i) for i in bal],
                        max_tracking_error_m=float(np.abs(eps).max()),
                        config=spec["cfg"])
            sc.append(info)
        index["shows"].append({
            "id": name, "name": show["name"], "n": int(n), "frames": int(K),
            "duration_s": float(fleet.t_end), "d_min_m": show["limits"]["min_separation_m"],
            "geofence": show["limits"]["geofence"], "crowd": show["limits"]["crowd"],
            "fleet_sha256": fleet.content_hash(), "verdict": cert["verdict"],
            "min_separation_m": rep.min_separation, "independent_recheck": independent_ok,
            "certificate": f"{name}/certificate.json", "scenarios": sc})
        print(name, n, K, cert["verdict"], round(rep.min_separation, 3), flush=True)
    (OUT / "index.json").write_text(json.dumps(round_floats(index), indent=1))


if __name__ == "__main__":
    main()
