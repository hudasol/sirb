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
SHOWS = ["gallery_120", "oryx_500", "torus_300", "sirb_text_100"]

# Display names for the stages of each show (stage ids come from shows/*.json). Arabic wording is
# new and awaits native review (docs/VIEWER.md).
STAGE_NAMES = {
    "launch": ("Launch", "الإقلاع"), "land": ("Landing", "الهبوط"), "orb": ("Orb", "الكرة"),
    "word": ("Sirb wordmark", "شعار سرب"), "ring": ("Ring", "الحلقة"), "torus": ("Torus", "الطارة"),
    "super": ("Supershape", "الشكل الفائق"), "oryx": ("Arabian oryx", "المها العربي"),
    "falcon": ("Falcon", "الصقر"), "dhow": ("Dhow", "الداو"), "palm": ("Palm", "النخلة"),
    "hero": ("Hero", "البطل"), "heart": ("Heart", "القلب"), "skyline": ("Skyline", "الأفق"),
}
SHOW_BLURBS = {
    "gallery_120": ("120 drones: falcon, dhow, palm, hero, heart, wordmark and skyline.",
                    "١٢٠ طائرة: صقر وداو ونخلة وبطل وقلب وشعار وأفق."),
    "oryx_500": ("500 drones: orb, Arabian oryx and a supershape.",
                 "٥٠٠ طائرة: كرة ومها عربي وشكل فائق."),
    "torus_300": ("300 drones: torus and a supershape.", "٣٠٠ طائرة: طارة وشكل فائق."),
    "sirb_text_100": ("100 drones: orb, wordmark and ring.", "١٠٠ طائرة: كرة وشعار وحلقة."),
}

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


def stage_windows(show: dict, plan_info: dict, t_end: float) -> list[dict]:
    """Stage k: `t_start` is when the fleet leaves the previous formation (0 for the first stage),
    `t_arrive` when it has reached this one, `t_end` when its hold ends. Read from the planner's
    own transition list, so the windows are the ones the trajectories were built from."""
    out, tr = [], plan_info["transitions"]
    for k, st in enumerate(show["stages"]):
        t_start = 0.0 if k == 0 else float(tr[k - 1]["t_start"])
        t_arrive = 0.0 if k == 0 else float(tr[k - 1]["t_start"] + tr[k - 1]["window_s"])
        hold = float(st.get("hold_s", 0.0))
        en, ar = STAGE_NAMES[st["name"]]
        out.append({"id": st["name"], "name_en": en, "name_ar": ar, "t_start": t_start,
                    "t_arrive": t_arrive, "t_end": min(t_arrive + hold, t_end) if k < len(show["stages"]) - 1 else t_end})
    return out


def evidence() -> dict:
    """Measured results shown on the viewer's 'Why Sirb' card, copied from the committed result files."""
    base = json.loads(Path("results/baselines.json").read_text())["sampled_false_pass"]
    b2 = json.loads(Path("results/benchmark_v2.json").read_text())["strata"]
    razor = b2["razor_violating"]
    return {
        "crossings": {"trials": base["trials"], "false_pass_rate": base["false_pass_rate"]},
        "razor": {"n": razor["n"], "exact_wrong": razor["exact_wrong"]["count"],
                  "frame_wrong_rate": {k: v["rate"] for k, v in razor["fixed_wrong"].items()}},
        "sources": ["results/baselines.json", "results/benchmark_v2.json"],
    }


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
             "evidence": evidence(), "shows": []}
    for name in SHOWS:
        show = load_show(f"shows/{name}.json")
        fleet, plan_info = plan_baseline(show)
        rep = verify(fleet, show["limits"])
        cert = make_certificate(rep, fleet, show["limits"])
        recomputation_ok = bool(verify_certificate(cert, fleet))
        ref = sample_reference(fleet, DT)
        K, n, _ = ref.shape
        d = OUT / name
        d.mkdir(exist_ok=True)
        (d / "certificate.json").write_text(json.dumps(cert, indent=1))
        sc = [{"id": "nominal", "label_en": "Nominal (passed the separation check)",
               "label_ar": "الخطة الاسمية (اجتازت فحص التباعد)", **write_bin(d / "nominal.bin", ref)}]
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
            "id": name, "name": show["name"],
            "blurb_en": SHOW_BLURBS[name][0], "blurb_ar": SHOW_BLURBS[name][1],
            "stages": stage_windows(show, plan_info, float(fleet.t_end)), "n": int(n), "frames": int(K),
            "duration_s": float(fleet.t_end), "d_min_m": show["limits"]["min_separation_m"],
            "geofence": show["limits"]["geofence"], "crowd": show["limits"]["crowd"],
            "fleet_sha256": fleet.content_hash(), "verdict": cert["verdict"],
            "min_separation_m": rep.min_separation, "recomputation_ok": recomputation_ok,
            "certificate": f"{name}/certificate.json", "scenarios": sc})
        print(name, n, K, cert["verdict"], round(rep.min_separation, 3), flush=True)
    (OUT / "index.json").write_text(json.dumps(round_floats(index), indent=1))


if __name__ == "__main__":
    main()
