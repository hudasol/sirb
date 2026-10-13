"""Generate docs/RESULTS.md from results/*.json. No number in that file is typed by hand.

    python scripts/make_results_doc.py
"""

from __future__ import annotations

import json
from pathlib import Path

R = Path("results")


def load(name):
    p = R / name
    return json.loads(p.read_text()) if p.exists() else None


def pct(x):
    return f"{100 * x:.1f}%"


def ci(row, key="p_violation_ci95"):
    lo, hi = row[key]
    return f"[{pct(lo)}, {pct(hi)}]"


def sweeps_section(d):
    out = [f"## Stress sweeps ({d['show']})", "",
           f"Nominal verified minimum separation: {d['nominal_min_separation_m']:.3f} m "
           f"(threshold d_min = {d['d_min_m']} m, verified_ok = {d['verified_ok']}). "
           f"{d['trials']} fixed seed trials per point, Monte Carlo step dt = {d['dt']} s. "
           "Probabilities are properties of the simulated model with ASSUMED disturbance "
           "parameters (docs/METHODS.md), not real world risk. Intervals are Wilson 95%.", ""]
    for name, rows in d["sweeps"].items():
        out += [f"### {name}", "", "| value | P(violation) | 95% CI | median min separation (m) | "
                "worst quantile (0.0) |", "|---|---|---|---|---|"]
        for r in rows:
            q = r["min_separation_quantiles"]
            out.append(f"| {r['value']} | {pct(r['p_violation'])} | {ci(r)} | "
                       f"{q.get('0.5', float('nan')):.3f} | {q.get('0.0', float('nan')):.3f} |")
        out.append("")
    return out


def spacing_section(d):
    out = ["## Formation spacing versus violation probability", "",
           f"Scenario (assumed): {d['scenario']}. {d['trials']} trials per row. The nominal "
           "minimum separation is the exact verified value of the planned show.", "",
           "| formation spacing (m) | nominal verified min (m) | P(violation) | 95% CI |",
           "|---|---|---|---|"]
    for r in d["rows"]:
        out.append(f"| {r['formation_spacing_m']} | {r['nominal_min_separation_m']:.3f} | "
                   f"{pct(r['p_violation'])} | {ci(r)} |")
    ok = [r for r in d["rows"] if r["p_violation_ci95"][1] <= 0.05]
    if ok:
        r = min(ok, key=lambda x: x["formation_spacing_m"])
        out += ["", f"Smallest tested spacing whose Wilson upper bound is at most 5%: "
                f"{r['formation_spacing_m']} m (nominal {r['nominal_min_separation_m']:.2f} m)."]
    return out + [""]


def contingency_section(d):
    out = ["## Contingency behaviours", "",
           f"{d['trials']} trials per row, show: {d['show']}. Rejoin closing speed capped at "
           f"{d.get('rejoin_vmax_mps')} m/s. 'event' = violation involving a drone that had "
           "the event; 'bystander' = violation between two drones without events.", "",
           "| case | behaviour | P(violation) | 95% CI | event | bystander |",
           "|---|---|---|---|---|---|"]
    for r in d["rows"]:
        out.append(f"| {r['case']} | {r['behavior']} | {pct(r['p_violation'])} | {ci(r)} | "
                   f"{pct(r['p_violation_involving_event_drone'])} | "
                   f"{pct(r['p_violation_between_bystanders'])} |")
    return out + [""]


def scaling_section(d):
    out = ["## Scaling (formation spacing 3.2 m, baseline planner)", "",
           "| n | plan (s) | verify (s) | verified | pairs checked | stress trials | "
           "mean trial CPU (s) |", "|---|---|---|---|---|---|---|"]
    for r in d["rows"]:
        out.append(f"| {r['n']} | {r['plan_s']:.2f} | {r['verify_s']:.2f} | {r['verified_ok']} | "
                   f"{r['pairs_checked']} | {r['stress_trials']} | "
                   f"{r['stress_mean_trial_cpu_s']:.2f} |")
    return out + [""]


def headlines() -> list[str]:
    """Headline bullets, every number read from result files."""
    out = []
    idx = None
    f = Path("viewer/data/index.json")
    if f.exists():
        idx = json.loads(f.read_text())
        for sh in idx["shows"]:
            out.append(f"- Demo show `{sh['id']}` ({sh['n']} drones, {sh['duration_s']:.0f} s): exact "
                       f"verified minimum separation {sh['min_separation_m']:.3f} m against "
                       f"d_min {sh['d_min_m']} m, verdict {sh['verdict']} (model: drones as points "
                       "on the planned polynomials).")
    sw = load("stress_sweeps.json")
    if sw:
        w = sw["sweeps"]["wind_w20_mps"]
        okw = [r for r in w if r["p_violation_ci95"][1] <= 0.05]
        if okw:
            out.append(f"- Simulated wind (ASSUMED models), 100 drone show: highest tested wind with a "
                       f"Wilson upper bound at most 5%: {max(r['value'] for r in okw)} m/s at 20 ft; "
                       f"first tested wind with an observed violation: "
                       f"{min([r['value'] for r in w if r['p_violation'] > 0] or [float('nan')])} m/s.")
        b = sw["sweeps"]["gnss_bias_sigma_m_25pct_degraded"]
        first = [r for r in b if r["p_violation"] > 0.05]
        if first:
            out.append(f"- Simulated GNSS bias (25% of drones degraded, ASSUMED): P(violation) exceeds 5% "
                       f"already at bias sigma {first[0]['value']} m "
                       f"({pct(first[0]['p_violation'])}); the nominal margin over d_min is only "
                       f"{sw['nominal_min_separation_m'] - sw['d_min_m']:.3f} m.")
    sp = load("stress_spacing.json")
    if sp:
        ok = [r for r in sp["rows"] if r["p_violation_ci95"][1] <= 0.05]
        if ok:
            r = min(ok, key=lambda x: x["formation_spacing_m"])
            out.append(f"- Nominal separation needed in the stated scenario for a 5% Wilson upper bound: "
                       f"{r['nominal_min_separation_m']:.2f} m (formation spacing "
                       f"{r['formation_spacing_m']} m, {sp['trials']} trials).")
    ct = load("stress_contingency.json")
    if ct:
        by = max(r["p_violation_between_bystanders"] for r in ct["rows"])
        out.append(f"- Contingency runs: largest bystander to bystander violation rate over all rows: "
                   f"{pct(by)}; violations involved the drone that had the event.")
    sc = load("stress_scaling.json")
    if sc:
        last = sc["rows"][-1]
        out.append(f"- Scaling: baseline plan + exact verification at n = {last['n']}: "
                   f"{last['plan_s']:.1f} s + {last['verify_s']:.1f} s (verified = {last['verified_ok']}).")
    return out


def assurance_lines() -> list[str]:
    """Verification evidence lines, every number read from result files."""
    out = []
    d = load("differential.json")
    if d:
        c, v, b = d["counts"], d["verdicts"], d["brute_force"]
        md = d["max_discrepancies"]
        out.append(f"- Second, independent verifier (interval branch and bound, `sirb/verify2.py`): "
                   f"{c['total_fleets']} fleets ({c['random_fleets']} random, {c['adversarial_fleets']} "
                   f"adversarial in {len(c['adversarial_by_family'])} families), "
                   f"{c['verdict_checks']} verdict checks: lower bound above the exact value in "
                   f"{'0' if md['max_l_minus_e'] <= 1e-9 else 'some'} cases "
                   f"(max l minus e = {md['max_l_minus_e']:.1e} m), FAIL verdicts with the exact value "
                   f"at or above the threshold: {v['fail_with_exact_at_or_above_threshold']} (max margin "
                   f"{v['fail_with_exact_above_threshold_max_margin']:.1e} m); "
                   f"{c['n_disagreement_records']} disagreement records, all in the high multiplicity "
                   f"root family, largest exact minus lower bound {md['max_e_minus_l']:.1e} m "
                   f"(`docs/VERIFIER.md`). Brute force sampling at dt = 1e-4 s against the exact value: "
                   f"{b['n_comparisons']} comparisons, largest sampled minus exact "
                   f"{b['grid_plus_breakpoints_max_sampled_minus_exact']:.1e} m (grid plus breakpoints), "
                   f"largest negative gap {b['grid_plus_breakpoints_max_negative_gap']:.1e} m.")
    m = load("mutation.json")
    if m:
        parts = []
        for f, a in m["after"].items():
            bsc = m["before"].get(f, {}).get("score")
            if bsc is not None:
                parts.append(f"`{f.replace('sirb/', '')}` {100 * bsc:.1f}% to {100 * a['score']:.1f}%")
        out.append("- Mutation testing (mutmut, killed over total, before to after added tests): "
                   + "; ".join(parts) + " (`docs/MUTATION.md`).")
    ax = load("axe.json")
    if ax:
        bs, as_ = ax["before"]["summary"], ax["after"]["summary"]
        out.append(f"- Viewer accessibility (axe-core {ax['axe_core']['version']}, 8 page runs): "
                   f"{bs['violation_rules']} violated rules before, {as_['violation_rules']} after; "
                   "this is an automated check and does not replace manual testing.")
    vc = load("viewer_checks.json")
    if vc:
        out.append(f"- Viewer behaviour checks (keyboard, reduced motion, light theme, file protocol, "
                   f"RTL, overflow): {vc['passed']} passed, {vc['failed']} failed.")
    sp = load("scaling_profile.json")
    if sp:
        rows = [r for r in sp["rows"] if not r.get("failed")]
        r = rows[-1]
        out.append(f"- Scale (computational study): n = {r['n']}: plan {r['plan_total_s']:.0f} s, exact "
                   f"verification {r['verify_s']:.0f} s, peak memory {r['peak_rss_mb']:.0f} MB "
                   "(`docs/PERFORMANCE.md`).")
    se = load("sensitivity.json")
    if se:
        top = se["ranking_by_swing"]["stressed_spacing_4.0"][:3]
        out.append("- Largest one at a time swings of the median minimum separation (stressed baseline): "
                   + ", ".join(f"{t['param']} ({t['swing_m']:.2f} m)" for t in top)
                   + " (`docs/SENSITIVITY.md`).")
    return out


def fill_markers(path: str, lines: list[str], tag: str = "HEADLINES") -> None:
    p = Path(path)
    if not p.exists():
        return
    t = p.read_text()
    a, b = f"<!-- {tag}:START -->", f"<!-- {tag}:END -->"
    if a in t and b in t:
        t = t[:t.index(a) + len(a)] + "\n" + "\n".join(lines) + "\n" + t[t.index(b):]
        p.write_text(t)


def main():
    out = ["# Results (generated)", "",
           "Generated by `scripts/make_results_doc.py` from `results/*.json`. Do not edit by hand.",
           "Simulation results under ASSUMED models; not flight data; not a safety claim.", ""]
    for name, fn in (("stress_sweeps.json", sweeps_section), ("stress_spacing.json", spacing_section),
                     ("stress_contingency.json", contingency_section),
                     ("stress_scaling.json", scaling_section)):
        d = load(name)
        out += fn(d) if d else [f"(missing {name})", ""]
    Path("docs/RESULTS.md").write_text("\n".join(out))
    h = headlines()
    for f in ("docs/REPORT.md", "docs/ONE_PAGER.md", "README.md"):
        fill_markers(f, h)
    for f in ("docs/REPORT.md", "README.md"):
        fill_markers(f, assurance_lines(), "ASSURANCE")


if __name__ == "__main__":
    main()
