"""WP3: tuned PX4 SIH replay with identification of a simple tracking model (SITL, not hardware).

Predictions P45 to P47 in docs/PREDICTIONS.md were committed before this ran. For each of
2, 3 and 4 vehicles and show seeds 0 to 9, a planned and exactly verified show is streamed as
offboard setpoints to PX4 simulation in hardware (SIH) quadrotors twice: with PX4 default
parameters and with a tuned set of documented position and velocity parameters. A second order
loop with a transport delay is fitted to each flown log by least squares.

Start nothing by hand: the script starts and stops the PX4 instances itself
(scripts/sitl_instances.sh, PX4 built as in docs/SITL.md).

    python scripts/sitl_identify.py            # writes results/sitl_tuned.json, resumable
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from pymavlink import mavutil
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).parent))
import sitl_replay as base  # noqa: E402

from sirb.envinfo import environment  # noqa: E402
from sirb.planner.baseline import plan_baseline  # noqa: E402
from sirb.spec import new_show  # noqa: E402
from sirb.stress import exact_pair_min  # noqa: E402
from sirb.verify import verify  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / "results" / ".cache"
PX4_DIR = Path(os.environ.get("PX4_DIR", "/home/claude/px4work/PX4-Autopilot"))
SEEDS = list(range(10))
COUNTS = (2, 3, 4)
TUNED = {"MPC_Z_VEL_MAX_UP": 6.5, "MPC_Z_VEL_MAX_DN": 3.5, "MPC_ACC_UP_MAX": 6.0,
         "MPC_ACC_DOWN_MAX": 5.0, "MPC_ACC_HOR_MAX": 8.0, "MPC_ACC_HOR": 6.0,
         "MPC_XY_P": 1.5, "MPC_XY_VEL_P_ACC": 3.0, "MPC_Z_P": 1.5, "MPC_Z_VEL_P_ACC": 8.0,
         "MPC_TILTMAX_AIR": 60.0, "MPC_JERK_AUTO": 8.0}
CONFIGS = {"default": {}, "tuned": TUNED}
MAX_DEV_LIMIT_M = 3.0


def seeded_show(n: int, seed: int) -> dict:
    rng = np.random.default_rng(1000 + seed)
    sp = float(rng.uniform(4.5, 6.0))
    off = rng.uniform(-5.0, 5.0, size=2)
    grid = {"type": "launch_grid", "spacing_m": 4.0, "center": [float(off[0]), -30.0 + float(off[1]), 0.0]}
    return new_show(f"SITL tuned replay ({n} drones, seed {seed})", n, [
        {"name": "launch", "shape": grid, "hold_s": 2},
        {"name": "ring", "shape": {"type": "ring", "plane": "xz", "spacing_m": sp,
                                    "center": [0, 0, 20]}, "hold_s": 4},
        {"name": "land", "shape": grid, "hold_s": 2}], seed=seed)


def restart(n: int) -> None:
    sh = str(ROOT / "scripts" / "sitl_instances.sh")
    subprocess.run(["sh", sh, "stop"], check=False)
    subprocess.run(["sh", sh, "start", str(n)], check=True)
    time.sleep(10.0 + 2.0 * n)


def set_params(links, params: dict) -> bool:
    ok = True
    for m in links:
        for name, val in params.items():
            m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(val),
                                 mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            t0 = time.time()
            got = False
            while time.time() - t0 < 3.0 and not got:
                msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
                got = msg is not None and msg.param_id.rstrip("\x00") == name and abs(msg.param_value - val) < 1e-3
            ok &= got
    return ok


def fit_tracking(cmd: np.ndarray, vel: np.ndarray, flown: np.ndarray, dt: float):
    """Fit natural frequency, damping and delay of x'' = wn^2 (c - x) + 2 zeta wn (cdot - xdot)."""
    K = cmd.shape[0]
    C, V, X = (a.reshape(K, -1) for a in (cmd, vel, flown))

    def simulate(p):
        wn, ze, tau = p
        sub = 10
        h = dt / sub
        x, v = X[0].copy(), np.zeros_like(X[0])
        out = np.empty_like(X)
        out[0] = x
        for k in range(1, K):
            for s in range(sub):
                tt = (k - 1 + s / sub) * dt - tau
                i = int(np.clip(np.floor(tt / dt), 0, K - 1))
                a = wn * wn * (C[i] - x) + 2 * ze * wn * (V[i] - v)
                v = v + a * h
                x = x + v * h
            out[k] = x
        return out

    def resid(p):
        return (simulate(p) - X).ravel()

    fits = [least_squares(resid, x0=x0, bounds=([0.3, 0.2, 0.0], [20.0, 3.0, 1.0]))
            for x0 in ([1.0, 0.9, 0.05], [2.0, 0.9, 0.1], [4.0, 0.7, 0.2], [8.0, 1.0, 0.3])]
    r = min(fits, key=lambda f: f.cost)
    rms = float(np.sqrt(np.mean(r.fun ** 2)))
    return {"omega_n_rad_s": float(r.x[0]), "zeta": float(r.x[1]), "delay_s": float(r.x[2]),
            "rms_fit_error_m": rms}


def run_once(n: int, seed: int, cfg_name: str) -> dict:
    show = seeded_show(n, seed)
    fleet, _ = plan_baseline(show)
    rep = verify(fleet, show["limits"])
    T = fleet.t_end
    start = fleet.sample(np.array([0.0]))[0]
    out = {"n_vehicles": n, "seed": seed, "config": cfg_name, "plan_duration_s": T,
           "planned_min_separation_exact_m": rep.min_separation, "exact_verdict": "PASS" if rep.ok else "FAIL"}
    restart(n)
    links = [base.connect(i) for i in range(n)]
    out["params_set"] = set_params(links, CONFIGS[cfg_name]) if CONFIGS[cfg_name] else True
    hold0 = np.zeros(3)
    for _ in range(int(2 * base.RATE_HZ)):
        for m in links:
            base.setpoint(m, hold0, hold0)
        time.sleep(1 / base.RATE_HZ)
    armed = [False] * n
    t_arm = time.time()
    while not all(armed) and time.time() - t_arm < 30.0:
        for i, m in enumerate(links):
            if armed[i]:
                continue
            m.mav.command_long_send(m.target_system, m.target_component,
                                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
            m.mav.command_long_send(m.target_system, m.target_component,
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        for _ in range(int(base.RATE_HZ)):
            for i, m in enumerate(links):
                base.setpoint(m, hold0, hold0)
                while True:
                    hb = m.recv_match(type="HEARTBEAT", blocking=False)
                    if hb is None:
                        break
                    armed[i] = bool(hb.base_mode & 128)
            time.sleep(1 / base.RATE_HZ)
    out["armed"] = all(armed)
    if not all(armed):
        out["failure"] = "not all vehicles armed within 30 s"
        return out

    def at(t):
        a = fleet.sample(np.array([t]))[0] - start
        b = (fleet.sample(np.array([t + 0.05]))[0] - fleet.sample(np.array([max(t - 0.05, 0.0)]))[0]) / 0.1
        return a, b

    log = [[] for _ in links]
    t0 = time.time()
    step = 0
    while time.time() - t0 <= T + 3.0:
        t = time.time() - t0
        p, v = at(min(t, T))
        for i, m in enumerate(links):
            base.setpoint(m, base.to_ned(p[i]), base.to_ned(v[i]))
            while True:
                msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
                if msg is None:
                    break
                log[i].append((time.time() - t0, msg.x, msg.y, msg.z))
        step += 1
        time.sleep(max(0.0, step / base.RATE_HZ - (time.time() - t0)))
    grid = np.arange(0.0, T, 1 / base.RATE_HZ)
    plan = fleet.sample(grid)
    vel = np.gradient(plan, 1 / base.RATE_HZ, axis=0)
    flown = np.empty_like(plan)
    for i in range(n):
        L = np.array(log[i])
        if len(L) < 10:
            out["failure"] = f"no position log for vehicle {i}"
            return out
        ours = np.stack([L[:, 2], L[:, 1], -L[:, 3]], axis=1) + start[i]
        for ax in range(3):
            flown[:, i, ax] = np.interp(grid, L[:, 0], ours[:, ax])
    err = np.linalg.norm(flown - plan, axis=2)
    pairs = np.array([(a, b) for a in range(n) for b in range(a + 1, n)])
    pf, _ = exact_pair_min(flown, pairs)
    out.update({"flown_min_separation_m": float(pf.min()), "max_deviation_m": float(err.max()),
                "mean_error_m": float(err.mean()), "p95_error_m": float(np.percentile(err, 95)),
                "deviates": bool(err.max() > MAX_DEV_LIMIT_M),
                "below_threshold": bool(pf.min() < show["limits"]["min_separation_m"])})
    out["fit"] = fit_tracking(plan, vel, flown, 1 / base.RATE_HZ)
    return out


def main() -> None:
    CACHE.mkdir(parents=True, exist_ok=True)
    runs = []
    for cfg in CONFIGS:
        for n in COUNTS:
            for seed in SEEDS:
                f = CACHE / f"sitl_{cfg}_{n}_{seed}.json"
                if not f.exists():
                    f.write_text(json.dumps(run_once(n, seed, cfg)))
                runs.append(json.loads(f.read_text()))
                r = runs[-1]
                print(cfg, n, seed, r.get("max_deviation_m"), r.get("flown_min_separation_m"), flush=True)
    subprocess.run(["sh", str(ROOT / "scripts" / "sitl_instances.sh"), "stop"], check=False)
    good = [r for r in runs if "max_deviation_m" in r]

    def stats(cfg):
        g = [r for r in good if r["config"] == cfg]
        return {"runs": len(g), "failed_runs": sum(1 for r in runs if r["config"] == cfg) - len(g),
                "fraction_deviating": float(np.mean([r["deviates"] for r in g])) if g else None,
                "fraction_below_threshold": float(np.mean([r["below_threshold"] for r in g])) if g else None,
                "median_max_deviation_m": float(np.median([r["max_deviation_m"] for r in g])) if g else None,
                "median_omega_n": float(np.median([r["fit"]["omega_n_rad_s"] for r in g])) if g else None,
                "median_zeta": float(np.median([r["fit"]["zeta"] for r in g])) if g else None,
                "median_delay_s": float(np.median([r["fit"]["delay_s"] for r in g])) if g else None,
                "median_fit_rms_m": float(np.median([r["fit"]["rms_fit_error_m"] for r in g])) if g else None}
    st = {c: stats(c) for c in CONFIGS}
    px4_rev = subprocess.run(["git", "-C", str(PX4_DIR), "rev-parse", "HEAD"], capture_output=True,
                             text=True).stdout.strip()
    res = {"label": "PX4 simulation in hardware (SITL); not hardware; not flight evidence",
           "px4_commit": px4_rev, "tuned_parameters": TUNED, "seeds": SEEDS, "vehicle_counts": list(COUNTS),
           "max_deviation_limit_m": MAX_DEV_LIMIT_M, "runs": runs, "summary": st,
           "P45": {"tuned_fraction_deviating": st["tuned"]["fraction_deviating"],
                   "default_fraction_deviating": st["default"]["fraction_deviating"]},
           "P46": {"default_fraction_below_threshold": st["default"]["fraction_below_threshold"],
                   "tuned_fraction_below_threshold": st["tuned"]["fraction_below_threshold"]},
           "P47": {"median_omega_n_default": st["default"]["median_omega_n"],
                   "median_omega_n_tuned": st["tuned"]["median_omega_n"],
                   "median_fit_rms_tuned_m": st["tuned"]["median_fit_rms_m"]},
           "environment": environment({"show_seeds": SEEDS})}
    (ROOT / "results" / "sitl_tuned.json").write_text(json.dumps(res, indent=1))
    print(json.dumps({k: res[k] for k in ("summary", "P45", "P46", "P47")}, indent=1))


if __name__ == "__main__":
    main()
