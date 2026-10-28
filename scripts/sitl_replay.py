"""Offline replay of a small planned show through PX4 SIH (simulation in hardware), 2 to 4 vehicles.

Simulation only. Every PX4 instance runs in this workspace as a simulated multirotor; nothing here
connects to a real vehicle or radio. The planner and verifier are not touched: the planned polynomials
are sampled and streamed as offboard position and velocity setpoints, the simulated vehicles' local
positions are logged, and the flown paths are compared with the plan.

Start the instances first (see docs/SITL.md), then:   python scripts/sitl_replay.py 4
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import psutil
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.chdir(ROOT)

from sirb.planner.baseline import plan_baseline  # noqa: E402
from sirb.spec import new_show  # noqa: E402
from sirb.stress import exact_pair_min  # noqa: E402
from sirb.verify import verify  # noqa: E402

RATE_HZ = 20.0
LAUNCH = {"type": "launch_grid", "spacing_m": 4.0, "center": [0.0, -30.0, 0.0]}


def small_show(n: int) -> dict:
    return new_show(f"SITL replay ({n} drones)", n, [
        {"name": "launch", "shape": LAUNCH, "hold_s": 2},
        {"name": "ring", "shape": {"type": "ring", "plane": "xz", "spacing_m": 5.0,
                                    "center": [0, 0, 20]}, "hold_s": 4},
        {"name": "land", "shape": LAUNCH, "hold_s": 2},
    ], seed=7)


def connect(i: int):
    m = mavutil.mavlink_connection(f"udpin:0.0.0.0:{14540 + i}")
    m.wait_heartbeat(timeout=20)
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 32, 50000, 0, 0, 0, 0, 0)
    return m


def setpoint(m, p_ned, v_ned):
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3520,
        p_ned[0], p_ned[1], p_ned[2], v_ned[0], v_ned[1], v_ned[2], 0, 0, 0, 0, 0)


def to_ned(p):
    return np.array([p[1], p[0], -p[2]])       # our frame: x east, y north, z up


def main(n: int = 4) -> int:
    show = small_show(n)
    fleet, _ = plan_baseline(show)
    rep = verify(fleet, show["limits"])
    T = fleet.t_end
    start = fleet.sample(np.array([0.0]))[0]
    print(f"planned {n} drones, T = {T:.1f} s, exact min separation {rep.min_separation:.3f} m", flush=True)
    links = [connect(i) for i in range(n)]

    def at(t):
        a = fleet.sample(np.array([t]))[0] - start
        b = (fleet.sample(np.array([t + 0.05]))[0] - fleet.sample(np.array([max(t - 0.05, 0.0)]))[0]) / 0.1
        return a, b

    t_wall = time.time()
    hold0 = np.zeros(3)
    for _ in range(int(2 * RATE_HZ)):           # stream before switching to offboard
        for m in links:
            setpoint(m, hold0, hold0)
        time.sleep(1 / RATE_HZ)
    armed_ok = [False] * len(links)
    t_arm = time.time()
    while not all(armed_ok) and time.time() - t_arm < 30.0:      # retry until every vehicle is armed
        for i, m in enumerate(links):
            if armed_ok[i]:
                continue
            m.mav.command_long_send(m.target_system, m.target_component,
                                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)
            m.mav.command_long_send(m.target_system, m.target_component,
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        for _ in range(int(1 * RATE_HZ)):
            for i, m in enumerate(links):
                setpoint(m, hold0, hold0)
                while True:
                    hb = m.recv_match(type="HEARTBEAT", blocking=False)
                    if hb is None:
                        break
                    armed_ok[i] = bool(hb.base_mode & 128)
            time.sleep(1 / RATE_HZ)
    print("armed:", armed_ok, f"after {time.time() - t_arm:.1f} s", flush=True)
    if not all(armed_ok):
        print("not all vehicles armed; stopping")
        return 1

    log = [[] for _ in links]
    px4 = [p for p in psutil.process_iter(["name"]) if p.info["name"] == "px4"]
    for p in px4:
        p.cpu_percent(None)
    res = []
    t0 = time.time()
    step = 0
    while True:
        t = time.time() - t0
        if t > T + 3.0:
            break
        tp = min(t, T)
        p, v = at(tp)
        for i, m in enumerate(links):
            setpoint(m, to_ned(p[i]), to_ned(v[i]))
            while True:
                msg = m.recv_match(type="LOCAL_POSITION_NED", blocking=False)
                if msg is None:
                    break
                log[i].append((time.time() - t0, msg.x, msg.y, msg.z))
        step += 1
        if step % int(RATE_HZ) == 0:
            res.append((sum(p.cpu_percent(None) for p in px4) / 100.0,
                        [p.memory_info().rss / 1e6 for p in px4]))
        time.sleep(max(0.0, (step / RATE_HZ) - (time.time() - t0)))
    wall = time.time() - t_wall

    grid = np.arange(0.0, T, 1 / RATE_HZ)
    plan = fleet.sample(grid)
    flown = np.empty_like(plan)
    for i in range(n):
        L = np.array(log[i])
        if len(L) < 10:
            print("no position log for vehicle", i)
            return 1
        ours = np.stack([L[:, 2], L[:, 1], -L[:, 3]], axis=1) + start[i]   # back to x east, y north, z up
        for ax in range(3):
            flown[:, i, ax] = np.interp(grid, L[:, 0], ours[:, ax])
    err = np.linalg.norm(flown - plan, axis=2)
    pairs = np.array([(a, b) for a in range(n) for b in range(a + 1, n)])
    pf, _ = exact_pair_min(flown, pairs)
    pp, _ = exact_pair_min(plan, pairs)
    out = {"n_vehicles": n, "plan_duration_s": T, "wall_s": wall, "rate_hz": RATE_HZ,
           "d_min_m": show["limits"]["min_separation_m"],
           "planned_min_separation_exact_m": rep.min_separation,
           "planned_min_separation_on_replay_grid_m": float(pp.min()),
           "flown_min_separation_m": float(pf.min()),
           "tracking_error_m": {"mean": float(err.mean()), "p95": float(np.percentile(err, 95)),
                                "max": float(err.max())},
           "per_vehicle_max_error_m": [float(x) for x in err.max(axis=0)],
           "log_samples": [len(x) for x in log],
           "px4_processes": len(px4),
           "px4_cpu_cores_mean": float(np.mean([r[0] for r in res])),
           "px4_cpu_cores_max": float(np.max([r[0] for r in res])),
           "px4_rss_mb_max_per_instance": float(max(max(r[1]) for r in res)),
           "machine_cores": os.cpu_count()}
    (ROOT / "results" / f"sitl_replay_{n}.json").write_text(json.dumps(out, indent=1))
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main(int(sys.argv[1]) if len(sys.argv) > 1 else 4))
