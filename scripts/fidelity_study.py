"""Does a command latency or an acceleration cap in the tracker move the stress conclusions?

Paired design: the same seed and the same disturbance draws for every tracker model, so differences
come from the tracker only. Writes results/fidelity.json.   python scripts/fidelity_study.py
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
os.chdir(ROOT)

from run_stress import cached  # noqa: E402

from sirb.disturbance import StressConfig  # noqa: E402
from sirb.planner.baseline import plan_baseline  # noqa: E402
from sirb.spec import load_show  # noqa: E402
from sirb.stress import monte_carlo, sample_reference  # noqa: E402

SEED = 20261215
TRIALS = 100
DT = 0.1
WORKERS = int(os.environ.get("SIRB_WORKERS", "2"))

MODELS = {
    "baseline (linear, no limits)": dict(),
    "latency 0.2 s": dict(latency_s=0.2),
    "latency 0.3 s": dict(latency_s=0.3),
    "latency 0.4 s": dict(latency_s=0.4),
    "accel cap 4 m/s^2": dict(accel_limit=4.0),
    "accel cap 2 m/s^2": dict(accel_limit=2.0),
    "latency 0.2 s + accel cap 4 m/s^2": dict(latency_s=0.2, accel_limit=4.0),
}
CASES = {
    "GNSS bias 0.5 m, 25% degraded": dict(gnss_deg_fraction=0.25, gnss_bias_sigma=0.5),
    "wind 9 m/s": dict(w20=9.0),
    "comms loss 5%, corridor": dict(comms_loss_fraction=0.05,
                                    behavior={"comms_loss": "corridor", "battery_low": "descend",
                                              "gnss_loss": "continue", "motor_failure": "ballistic"}),
}


def delay_margin_s(cfg: StressConfig) -> dict:
    """Delay margin of the continuous loop  s^2 + (kd s + kp) e^(-sL) = 0  (feedback only)."""
    import math
    kp, kd = cfg.omega_n**2, 2 * cfg.zeta * cfg.omega_n
    # open loop L(s) = (kd s + kp) / s^2 ; gain crossover |L(jw)| = 1  ->  w^4 - kd^2 w^2 - kp^2 = 0
    w2 = (kd**2 + math.sqrt(kd**4 + 4 * kp**2)) / 2
    w = math.sqrt(w2)
    pm = math.atan2(kd * w, kp)          # phase margin, rad (phase of the PD zero)
    return {"crossover_rad_s": w, "phase_margin_deg": math.degrees(pm), "delay_margin_s": pm / w}


def required_feedback_accel(ref, case_kw, d_min, n_trials=20) -> dict:
    """Peak feedback acceleration norm the linear loop asks for, per drone, over trials."""
    import numpy as np

    from sirb.contingency import apply_events
    from sirb.disturbance import gnss_error, sample_events, wind_velocity
    from sirb.stress import track
    cfg = StressConfig(dt=DT, rejoin_vmax=10.0, **case_kw)
    kp, kd = cfg.omega_n**2, 2 * cfg.zeta * cfg.omega_n
    peaks = []
    for s in range(n_trials):
        rng = np.random.default_rng(SEED + 7919 + s)
        K, n, _ = ref.shape
        wind = wind_velocity(ref, cfg, rng)
        e, ginfo = gnss_error(K, n, cfg, rng)
        events = sample_events(K, n, cfg, rng)
        R, _ = apply_events(ref, events, ginfo, cfg)
        eps = track(R, -kp * e + cfg.drag_gain * wind, cfg)
        vel = np.gradient(eps, cfg.dt, axis=0)
        ctrl = np.linalg.norm(-kp * (eps + e) - kd * vel, axis=2)
        peaks.append(ctrl.max(axis=0))
    allp = np.concatenate(peaks)
    return {"drone_peaks_p50": float(np.percentile(allp, 50)), "drone_peaks_p95": float(np.percentile(allp, 95)),
            "drone_peaks_max": float(allp.max()), "trials": n_trials,
            "wind_drag_accel_mean_m_s2": float(cfg.drag_gain * cfg.w20 * (60.0 / 6.1) ** (1 / 7))}


def main():
    show = load_show("shows/sirb_text_100.json")
    fleet, _ = plan_baseline(show)
    d_min = show["limits"]["min_separation_m"]
    ref = sample_reference(fleet, DT)
    rows = []
    for case, ckw in CASES.items():
        for model, mkw in MODELS.items():
            t = time.time()
            mc = cached(f"fid_{case}_{model}_{DT}_{TRIALS}",
                        lambda ckw=ckw, mkw=mkw: monte_carlo(
                            ref, StressConfig(dt=DT, rejoin_vmax=10.0, **ckw, **mkw), d_min,
                            TRIALS, SEED, workers=WORKERS))
            rows.append({"case": case, "model": model, **mc})
            print(case, "|", model, round(mc["p_violation"], 3),
                  round(mc["min_separation_quantiles"]["0.5"], 3),
                  round(mc["mean_max_tracking_error_m"], 3), round(time.time() - t), "s", flush=True)
    authority = {case: required_feedback_accel(ref, ckw, d_min) for case, ckw in CASES.items()}
    for case, a in authority.items():
        print("authority", case, {k: round(v, 2) for k, v in a.items()}, flush=True)
    out = {"show": show["name"], "trials": TRIALS, "dt": DT, "seed": SEED,
           "delay_margin": delay_margin_s(StressConfig()), "required_feedback_accel": authority,
           "rows": rows}
    (ROOT / "results" / "fidelity.json").write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
