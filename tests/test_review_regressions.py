"""Regression tests for defects found in the critique rounds (docs/REVIEW_ROUNDS.md).

Each test names the round that found the defect.
"""

import copy
import json

import numpy as np
import pytest

from sirb import cli
from sirb.assign import linear_conflicts
from sirb.planner.baseline import plan_baseline
from sirb.spec import new_show, validate_show


def _show(n=12):
    return new_show("t", n, [
        {"name": "a", "shape": {"type": "launch_grid", "spacing_m": 4.0, "center": [0, -85, 0]},
         "hold_s": 1},
        {"name": "b", "shape": {"type": "sphere", "spacing_m": 4.0, "center": [0, 0, 60]},
         "hold_s": 2}], seed=3)


# ---- Round 1 (EDGE-style engineer) ---------------------------------------------------------

MALFORMED = {
    "geofence missing x": lambda s: s["limits"]["geofence"].pop("x"),
    "crowd missing y": lambda s: s["limits"]["crowd"].pop("y"),
    "geofence not an object": lambda s: s["limits"].__setitem__("geofence", [1, 2]),
    "stage not an object": lambda s: s["stages"].__setitem__(0, 5),
    "hold_s string": lambda s: s["stages"][0].__setitem__("hold_s", "2"),
    "z_max string": lambda s: s["limits"].__setitem__("z_max", "5"),
    "limits is a list": lambda s: s.__setitem__("limits", []),
    "stages is an object": lambda s: s.__setitem__("stages", {}),
    "inf limit": lambda s: s["limits"].__setitem__("amax", float("inf")),
    "nan limit": lambda s: s["limits"].__setitem__("vmax_h", float("nan")),
    "nan geofence": lambda s: s["limits"]["geofence"].__setitem__("x", [float("nan"), 3]),
    "threshold below vehicle diameter":
        lambda s: s["limits"].__setitem__("min_separation_m", 0.3),
    "vehicle not an object": lambda s: s.__setitem__("vehicle", 3),
}


@pytest.mark.parametrize("name", list(MALFORMED))
def test_round1_validator_rejects_malformed_without_raising(name):
    s = copy.deepcopy(_show())
    MALFORMED[name](s)
    errs = validate_show(s, check_targets=False)
    assert errs, name


def test_round1_validator_rejects_non_object():
    assert validate_show([1, 2]) == ["show must be a JSON object"]


def test_round1_valid_show_still_valid():
    assert validate_show(_show()) == []


def test_round1_linear_conflicts_block_equals_all_pairs():
    rng = np.random.default_rng(5)
    n = 150
    src = rng.uniform(0, 25, (n, 3))
    dst = rng.uniform(0, 25, (n, 3))
    perm = rng.permutation(n)
    v = dst[perm] - src
    # reference: explicit double loop with the closed-form minimum
    ref = 0
    for i in range(n):
        for j in range(i + 1, n):
            dp, dv = src[i] - src[j], v[i] - v[j]
            a = dv @ dv
            u = 0.0 if a < 1e-18 else min(max(-(dp @ dv) / a, 0.0), 1.0)
            if np.linalg.norm(dp + u * dv) < 3.0:
                ref += 1
    assert ref > 0
    for block in (7, 64, 256, 1000):
        assert linear_conflicts(src, dst, perm, 3.0, block=block) == ref


def test_round1_monte_carlo_serial_fallback_without_fork(monkeypatch):
    import sirb.stress as st
    from sirb.disturbance import StressConfig

    show = _show(10)
    fleet, _ = plan_baseline(show)
    ref = st.sample_reference(fleet, 0.5)
    cfg = StressConfig(dt=0.5, w20=3.0)
    a = st.monte_carlo(ref, cfg, 2.0, 4, 1, workers=1)
    monkeypatch.setattr(st, "_fork_available", lambda: False)
    b = st.monte_carlo(ref, cfg, 2.0, 4, 1, workers=2)   # would raise ValueError on Windows
    assert a["min_separation_quantiles"] == b["min_separation_quantiles"]


def test_round1_cli_validate_plan_check(tmp_path, capsys):
    from sirb.spec import save_show
    sp, out = tmp_path / "s.json", tmp_path / "e.json"
    save_show(_show(), sp)
    assert cli.main(["validate", str(sp)]) == 0
    assert cli.main(["plan", str(sp), "--out", str(out)]) == 0
    assert cli.main(["check", str(out)]) == 0
    text = capsys.readouterr().out
    assert "not an approval" in text and "PASS" in text
    b = json.loads(out.read_text())
    b["fleet"]["drones"][0][0]["coeffs"][0][0] += 1.0
    out.write_text(json.dumps(b))
    assert cli.main(["check", str(out)]) == 1          # tamper -> non-zero, no traceback
    assert cli.main(["validate", str(tmp_path / "missing.json")]) == 1


# ---- Round 2 (TII-style robotics researcher) -------------------------------------------------

def _seg(coeffs, t0, t1):
    from sirb.trajectory import Segment
    return Segment(t0, t1, np.array(coeffs, float))


def test_round2_acceleration_step_between_segments_is_flagged():
    """Velocity-continuous but acceleration-discontinuous path (infinite jerk) used to pass the
    jerk check, which only looks inside segments."""
    from sirb.spec import DEFAULT_LIMITS
    from sirb.trajectory import Fleet, Trajectory
    from sirb.verify import verify

    # seg 1: x = 0.5 t^2 (a = 1, v(1) = 1); seg 2: x = 0.5 + t (a = 0, v(0) = 1)
    s1 = _seg([[0, 0, 20], [0, 0, 0], [0.5, 0, 0]], 0.0, 1.0)
    s2 = _seg([[0.5, 0, 20], [1.0, 0, 0]], 1.0, 2.0)
    rep = verify(Fleet([Trajectory([s1, s2])]), DEFAULT_LIMITS)
    assert {v.kind for v in rep.violations} == {"accel_discontinuity"}
    assert not rep.ok
    # a smooth chain (a = 1 on both sides) is accepted
    s2b = _seg([[0.5, 0, 20], [1.0, 0, 0], [0.5, 0, 0]], 1.0, 2.0)
    assert verify(Fleet([Trajectory([s1, s2b])]), DEFAULT_LIMITS).ok


def test_round2_planner_output_has_no_acceleration_steps():
    from sirb.spec import DEFAULT_LIMITS
    from sirb.verify import verify

    show = _show(20)
    fleet, _ = plan_baseline(show)
    rep = verify(fleet, show["limits"])
    assert rep.ok and "accel_discontinuity" not in rep.violation_counts
    assert DEFAULT_LIMITS["jmax"] > 0


def test_round2_rejoin_default_is_unrestricted_and_cap_is_opt_in():
    from sirb.contingency import rejoin_reference
    from sirb.disturbance import StressConfig

    dt, K = 0.2, 600
    t = np.arange(K) * dt
    nom = np.stack([np.clip(10 * (t - 10), 0, 100), 0 * t, 0 * t + 50], axis=1)  # 100 m in 10 s
    k0, k1 = int(10 / dt), int(20 / dt)

    def peak(vmax):
        cfg = StressConfig(dt=dt, rejoin_vmax=vmax)
        R = nom.copy()
        rejoin_reference(nom, R, k0, k1, cfg)
        return float((np.linalg.norm(np.diff(R, axis=0), axis=1) / dt).max()), R

    assert StressConfig().rejoin_vmax is None
    p_default, _ = peak(None)
    p_cap, R = peak(10.0)
    assert p_default > 25.0                  # 100 m gap closed in 6 s: far above any airframe limit
    assert p_cap <= 10.0 + 0.1
    assert np.linalg.norm(R[-1] - nom[-1]) < 1e-9   # still rejoins the nominal path


def test_round2_candidate_pair_pruning_never_hides_a_violation():
    """dev_bound is a heuristic (6-sigma style) pad. Check it against all-pairs on stressed runs."""
    import itertools

    from sirb.disturbance import StressConfig
    from sirb.stress import candidate_pairs, dev_bound, exact_pair_min, realize, sample_reference

    show = _show(30)
    fleet, _ = plan_baseline(show)
    dt = 0.2
    ref = sample_reference(fleet, dt)
    n = ref.shape[1]
    allp = np.array(list(itertools.combinations(range(n), 2)))
    cfgs = [dict(gnss_deg_fraction=1.0, gnss_bias_sigma=2.0, w20=6.0),
            dict(gnss_loss_fraction=0.4, behavior={"comms_loss": "continue", "battery_low": "descend",
                                                   "gnss_loss": "rejoin", "motor_failure": "ballistic"})]
    violating = 0
    for kw in cfgs:
        cfg = StressConfig(dt=dt).with_(**kw)
        for seed in range(2):
            X, R, *_ = realize(ref, cfg, [9, seed])
            full, _ = exact_pair_min(X, allp)
            truth = {tuple(p) for p in allp[full < 2.0]}
            violating += len(truth)
            pairs, klo, khi = candidate_pairs(R, 2.0 + dev_bound(cfg), dt=dt)
            pm, _ = exact_pair_min(X, pairs, klo, khi)
            assert truth <= {tuple(p) for p in pairs[pm < 2.0]}
    assert violating > 0     # the test must actually exercise violations


# ---- Round 3 (Al Ain University senior: docs-code consistency, claims vs evidence) -----------

def test_round3_monte_carlo_is_independent_of_worker_count():
    """README/METHODS say results are deterministic given the seed; the runner may use 2 workers."""
    import sirb.stress as st
    from sirb.disturbance import StressConfig

    show = _show(10)
    fleet, _ = plan_baseline(show)
    ref = st.sample_reference(fleet, 0.5)
    cfg = StressConfig(dt=0.5, w20=6.0, gnss_deg_fraction=0.5, gnss_bias_sigma=0.5)
    a = st.monte_carlo(ref, cfg, 2.0, 6, 3, workers=1)
    b = st.monte_carlo(ref, cfg, 2.0, 6, 3, workers=2)
    for k in ("p_violation", "p_violation_ci95", "min_separation_quantiles",
              "mean_max_tracking_error_m", "mean_violating_pairs"):
        assert a[k] == b[k], k


def test_round3_required_value_needs_all_larger_rows_to_qualify():
    from sirb.stress import required_value, wilson

    def row(v, k, n=300):
        return {"formation_spacing_m": v, "p_violation_ci95": list(wilson(k, n))}

    rows = [row(3.0, 120), row(4.0, 30), row(5.0, 0), row(6.0, 4), row(8.0, 0)]
    # 5.0 is clean, but 6.0 (4/300) has an upper bound above 2%, so only 8.0 qualifies at 2%
    assert required_value(rows, 0.02)["value"] == 8.0
    assert required_value(rows, 0.05)["value"] == 5.0
    # below the best attainable bound for N = 300 nothing can qualify
    assert required_value(rows, 0.001)["value"] is None


def test_round3_docs_list_the_shapes_and_letters_the_code_supports():
    import re
    from pathlib import Path

    from sirb.shapes import SUPPORTED_LETTERS
    from sirb.spec import SHAPE_TYPES

    doc = Path("docs/SHOW_SCHEMA.md").read_text(encoding="utf-8")
    line = next(ln for ln in doc.splitlines() if ln.startswith("Shape types:"))
    documented = set(re.findall(r"`([a-z_]+)`", line.split("Common keys")[0]))
    assert SHAPE_TYPES <= documented
    assert set(SUPPORTED_LETTERS) == set(re.findall(r"letters ([A-Z ]+)\)", line)[0].replace(" ", ""))


def test_round3_methods_quotes_current_stress_defaults():
    from pathlib import Path

    from sirb.disturbance import StressConfig

    cfg = StressConfig()
    doc = Path("docs/METHODS.md").read_text(encoding="utf-8")
    for needle in (f"omega_n = {cfg.omega_n:g} rad/s", f"zeta = {cfg.zeta:g}",
                   f"drag_gain = {cfg.drag_gain:g}", f"{cfg.gnss_white_sigma:g} m per axis",
                   f"{cfg.gnss_dead_reckoning_rate:g} m/sqrt(s)",
                   f"decay {cfg.air_drag_per_s:g} /s"):
        assert needle in doc, needle
    assert "2.0 m centre-to-centre separation threshold is an **ASSUMPTION**" in doc


def test_round3_spec_labels_separation_threshold_as_assumption():
    from sirb.spec import DEFAULT_LIMITS
    assert DEFAULT_LIMITS["sources"]["min_separation_m"] == "assumption"


# ---- Round 4 (aviation safety assessor: wording, overclaiming, misreadable output) -----------

def test_round4_certificate_is_incomplete_when_a_limit_family_is_missing():
    import copy

    from sirb.certificate import make_certificate, verify_certificate
    from sirb.verify import verify

    show = _show(12)
    fleet, _ = plan_baseline(show)
    lim = copy.deepcopy(show["limits"])
    del lim["vmax_h"]
    lim["jmax"] = None
    rep = verify(fleet, lim)
    assert rep.ok                      # nothing was checked against the missing limits...
    cert = make_certificate(rep, fleet, lim)
    assert cert["verdict"] == "INCOMPLETE"   # ...so the certificate must not say PASS
    assert cert["unchecked_limits"] == ["vmax_h", "jmax"]
    assert verify_certificate(cert, fleet)
    cert["verdict"] = "PASS"
    assert not verify_certificate(cert, fleet)
    full = make_certificate(verify(fleet, show["limits"]), fleet, show["limits"])
    assert full["verdict"] == "PASS" and full["unchecked_limits"] == []


def test_round4_verdict_failure_still_fail_not_incomplete():
    from sirb.certificate import verdict_for
    assert verdict_for(False, {}) == "FAIL"


def _i18n_blocks():
    from pathlib import Path
    s = Path("viewer/i18n.js").read_text(encoding="utf-8")
    return s[s.index("  en: {"):s.index("  ar: {")], s[s.index("  ar: {"):s.index("export const CERT_AR")]


def test_round4_every_english_ui_string_has_an_arabic_counterpart():
    import re
    en, ar = _i18n_blocks()
    ke = set(re.findall(r"^    (\w+):", en, re.M))
    ka = set(re.findall(r"^    (\w+):", ar, re.M))
    assert ke == ka, (ke - ka, ka - ke)
    assert {"verdict_note", "scen_scope"} <= ke


def test_round4_no_ui_line_claims_safety_or_approval_without_negation():
    """Every user-facing line that uses a loaded word must also negate it in the same line."""
    import re
    from pathlib import Path
    loaded = re.compile(r"\b(safe|safely|approved|cleared|clearance|certified)\b", re.I)
    negation = re.compile(r"\b(not|no|nor|never|without|need|needs)\b|n't", re.I)
    offenders = []
    for f in ("viewer/i18n.js", "viewer/index.html", "viewer/methods.html", "viewer/app.js"):
        for i, line in enumerate(Path(f).read_text(encoding="utf-8").splitlines(), 1):
            if loaded.search(line) and not negation.search(line):
                offenders.append((f, i, line.strip()[:100]))
    assert not offenders, offenders


def test_round4_viewer_does_not_call_a_same_code_rerun_independent():
    import json
    from pathlib import Path
    for f in Path("viewer").rglob("*"):
        if f.suffix in {".js", ".html", ".json"} and "vendor" not in f.parts:
            text = f.read_text(encoding="utf-8")
            assert "independent_recheck" not in text, f
            assert "Independent recheck" not in text, f
    idx = json.loads(Path("viewer/data/index.json").read_text())
    assert all(isinstance(s["recomputation_ok"], bool) for s in idx["shows"])


def test_round4_cli_and_export_say_not_an_approval(tmp_path, capsys):
    from sirb.spec import save_show
    sp, out = tmp_path / "s.json", tmp_path / "e.json"
    save_show(_show(), sp)
    cli.main(["plan", str(sp), "--out", str(out)])
    text = capsys.readouterr().out
    assert "declared in the show file only" in text and "not an approval" in text
    b = json.loads(out.read_text())
    assert b["label"] == "simulation-validated only"
    assert "not flight-certified" in b["certificate"]["statement"]
    assert "Regulatory" in " ".join(b["certificate"]["not_guaranteed"])


# ---- Round 5 (ATRC-style engineer: boundaries, independence, test quality) -------------------

def _import_graph():
    """Module -> set of sirb modules it imports (static, including function-level imports)."""
    import ast
    from pathlib import Path

    root = Path("sirb")
    graph = {}
    for f in root.rglob("*.py"):
        parts = list(f.with_suffix("").parts)
        if parts[-1] == "__init__":
            parts = parts[:-1]
        mod = ".".join(parts)
        pkg = parts if f.name == "__init__.py" else parts[:-1]
        deps = set()
        for node in ast.walk(ast.parse(f.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Import):
                deps |= {a.name for a in node.names if a.name.split(".")[0] == "sirb"}
            elif isinstance(node, ast.ImportFrom):
                if node.level:
                    base = pkg[: len(pkg) - (node.level - 1)]
                    target = ".".join(base + ([node.module] if node.module else []))
                else:
                    target = node.module or ""
                if target.split(".")[0] == "sirb":
                    deps.add(target)
                    for a in node.names:             # `from . import X` may import a submodule
                        deps.add(f"{target}.{a.name}")
        graph[mod] = {d for d in deps}
    names = set(graph)
    return {m: {d for d in ds if d in names and d != m} for m, ds in graph.items()}


def _closure(graph, start):
    seen, todo = set(), [start]
    while todo:
        m = todo.pop()
        for d in graph.get(m, ()):
            if d not in seen:
                seen.add(d)
                todo.append(d)
    return seen


def test_round5_verifier_imports_nothing_from_sirb():
    g = _import_graph()
    assert g["sirb.verify"] == set(), g["sirb.verify"]


def test_round5_planners_and_stress_never_reach_the_verifier_or_certificate():
    g = _import_graph()
    judges = {"sirb.verify", "sirb.certificate", "sirb.export", "sirb.preflight", "sirb.cli"}
    producers = ["sirb.assign", "sirb.shapes", "sirb.spec", "sirb.trajectory", "sirb.stress",
                 "sirb.disturbance", "sirb.contingency"] + [m for m in g if m.startswith("sirb.planner")]
    for m in producers:
        reach = _closure(g, m)
        assert not (reach & judges), (m, reach & judges)


def test_round5_trajectory_module_depends_on_nothing():
    assert _import_graph()["sirb.trajectory"] == set()


def test_round5_import_graph_detects_a_forbidden_import(tmp_path, monkeypatch):
    """The checker itself must be able to fail: plant a forbidden import and look for it."""
    import shutil
    shutil.copytree("sirb", tmp_path / "sirb", ignore=shutil.ignore_patterns("__pycache__"))
    p = tmp_path / "sirb" / "planner" / "baseline.py"
    p.write_text(p.read_text() + "\nfrom ..verify import verify  # planted\n")
    monkeypatch.chdir(tmp_path)
    g = _import_graph()
    assert "sirb.verify" in _closure(g, "sirb.planner.baseline")


def test_round5_verifier_tests_have_power_grid_only_root_finding_is_caught(monkeypatch):
    """Mutation check: if the exact root finding were replaced by the safety grid alone, a
    minimum that falls between grid points must be missed, and the exact verifier must not."""
    import sirb.verify as V
    from sirb.trajectory import Fleet, Trajectory

    # relative motion linear: d(s) = |(-1.3 + 4 s, 0.5, 0)| on s in [0, 1] -> minimum at s = 0.325
    a = Trajectory([_seg([[0, 0, 20], [4, 0, 0]], 0.0, 1.0)])
    b = Trajectory([_seg([[1.3, 0.5, 20], [0, 0, 0]], 0.0, 1.0)])
    fl = Fleet([a, b])
    exact = V.pair_minima(fl, 10.0)[0][(0, 1)][0]
    assert exact == pytest.approx(0.5, abs=1e-9)

    real = V._critical_points
    monkeypatch.setattr(V, "_critical_points",
                        lambda df: np.broadcast_to(np.linspace(0, 1, V._GRID + 1),
                                                   (df.shape[0], V._GRID + 1)).copy())
    mutated = V.pair_minima(fl, 10.0)[0][(0, 1)][0]
    assert mutated > exact + 1e-3, "suite would not notice a verifier without root finding"
    monkeypatch.setattr(V, "_critical_points", real)
    assert V.pair_minima(fl, 10.0)[0][(0, 1)][0] == pytest.approx(exact, abs=1e-12)


def test_round5_clean_pair_reports_exact_minimum_against_dense_sampling():
    """Replaces a disjunctive assertion that held for any clean fleet."""
    from sirb.spec import DEFAULT_LIMITS
    from sirb.trajectory import Fleet, Trajectory
    from sirb.verify import verify

    def q(p0, p1, t0, T):
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
        d = p1 - p0
        c = np.zeros((6, 3))
        c[0], c[3], c[4], c[5] = p0, 10 * d / T**3, -15 * d / T**4, 6 * d / T**5
        return _seg(c, t0, t0 + T)

    a = Trajectory([q([0, 0, 20], [20, 0, 20], 0, 10), q([20, 0, 20], [20, 20, 20], 10, 10)])
    b = Trajectory([q([0, 10, 25], [20, 10, 25], 0, 10), q([20, 10, 25], [20, 30, 25], 10, 10)])
    fl = Fleet([a, b])
    ts = np.linspace(0, 20, 200001)
    truth = np.linalg.norm(fl.trajectories[0].sample(ts) - fl.trajectories[1].sample(ts), axis=1).min()
    rep = verify(fl, DEFAULT_LIMITS, search_radius=50.0)
    assert rep.ok and not rep.min_separation_is_lower_bound
    assert rep.min_separation == pytest.approx(truth, abs=1e-6)
    assert rep.min_separation <= truth + 1e-9          # exact value can never exceed the sampled one
    # default radius (8 m) is below the true minimum, so the report must say "lower bound"
    short = verify(fl, DEFAULT_LIMITS)
    assert short.min_separation_is_lower_bound and short.min_separation == 8.0


def test_round5_export_rejects_certificate_for_other_limits_and_malformed_structure(tmp_path):
    from sirb.export import build_export, load_export, save_export

    show = _show(12)
    fleet, _ = plan_baseline(show)
    good = build_export(show, fleet)

    def write(b):
        p = tmp_path / "x.json"
        save_export(p, b)
        return p

    load_export(write(good))
    loose = copy.deepcopy(good)
    loose["show"]["limits"]["min_separation_m"] = 1.0          # looser show, old certificate
    with pytest.raises(ValueError, match="different limits"):
        load_export(write(loose))
    for mutate in (lambda b: b.__setitem__("fleet", []),
                   lambda b: b.__setitem__("certificate", None),
                   lambda b: b.pop("show"),
                   lambda b: b["show"].__setitem__("n_drones", 99),
                   lambda b: b["show"]["limits"].__setitem__("geofence", 5)):
        bad = copy.deepcopy(good)
        mutate(bad)
        with pytest.raises(ValueError):
            load_export(write(bad))
    p = tmp_path / "list.json"
    p.write_text("[1, 2]")
    with pytest.raises(ValueError):
        load_export(p)


def test_round5_monte_carlo_does_not_retain_the_reference_array():
    import sirb.stress as st
    from sirb.disturbance import StressConfig

    show = _show(10)
    fleet, _ = plan_baseline(show)
    ref = st.sample_reference(fleet, 0.5)
    st.monte_carlo(ref, StressConfig(dt=0.5), 2.0, 2, 1)
    assert "ref" not in st._WORK


def test_round5_preflight_fails_closed_on_missing_or_nan_inputs():
    """NaN comparisons are False, so NaN centre/heading/wind used to read as 'no mismatch'."""
    from sirb.preflight import nominal_declaration, preflight

    show = _show(12)
    fleet, _ = plan_baseline(show)
    nan = float("nan")
    cases = [
        lambda d, ld: ld.update(center_xy=[nan, 0.0]),
        lambda d, ld: ld.update(heading_deg=nan),
        lambda d, ld: ld.pop("center_xy"),
        lambda d, ld: d.update(heading_deg="north"),
        lambda d, ld: ld.update(center_xy=[1.0]),
    ]
    for mutate in cases:
        d, ld = nominal_declaration(show, fleet)
        d, ld = copy.deepcopy(d), copy.deepcopy(ld)
        mutate(d, ld)
        rep = preflight(show, fleet, d, ld)
        assert not rep.ok and "INPUT_INVALID" in rep.codes("FAIL")
    d, ld = nominal_declaration(show, fleet)
    assert preflight(show, fleet, d, ld).ok
    for kw in ({"wind_forecast_mps": nan, "wind_limit_mps": 9.0},
               {"wind_forecast_mps": 4.0, "wind_limit_mps": 0.0}):
        assert "INPUT_INVALID" in preflight(show, fleet, d, ld, **kw).codes("FAIL")
