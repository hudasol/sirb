"""Generate docs/SENSITIVITY.md from results/sensitivity.json. No hand typed numbers."""

from __future__ import annotations

import json
from pathlib import Path

R = json.loads(Path("results/sensitivity.json").read_text())


def f(x, n=3):
    return f"{x:.{n}f}"


def main() -> None:
    L = ["# Sensitivity of the stress results", "",
         "Generated from `results/sensitivity.json` by `scripts/make_sensitivity_doc.py`. "
         "Everything here is a property of the simulated model with assumed parameters "
         "(see `docs/METHODS.md`). It says which assumptions the stress results depend on, "
         "not what a real fleet would do.", "",
         f"Scenario base: {R['scenario']}. One at a time (OAT) runs use {R['trials_oat']} trials "
         "per configuration with common random numbers (same trial seeds in every configuration, "
         "so differences are paired). Ranges are about half to double the default.", ""]
    for name, rows in R["ranking_by_swing"].items():
        d = R["oat"][name]
        b = d["baseline"]
        L += [f"## One at a time, {name}", "",
              f"Nominal planned minimum separation {f(d['nominal_min_separation_m'])} m, required "
              f"{f(d['d_min'], 2)} m. Baseline P(violation) {f(b['p'])} (95% CI "
              f"{f(b['ci'][0])} to {f(b['ci'][1])}), median minimum separation "
              f"{f(b['median_min_sep'])} m.", "",
              "| Parameter | Low | High | Delta median sep, low (m) | Delta median sep, high (m) "
              "| Swing (m) | P(viol) low | P(viol) high |", "|---|---|---|---|---|---|---|---|"]
        for r in rows:
            v = d["params"][r["param"]]
            L.append(f"| {r['param']} | {v['low']['value']} | {v['high']['value']} | "
                     f"{r['delta_median_min_sep_low']:+.3f} | {r['delta_median_min_sep_high']:+.3f} "
                     f"| {f(r['swing_m'])} | {f(v['low']['p'])} | {f(v['high']['p'])} |")
        L += ["", "Add on hazards switched on from zero:", "",
              "| Add on | P(viol) | 95% CI | Median min sep (m) |", "|---|---|---|---|"]
        for a, v in d["addons"].items():
            L.append(f"| {a} | {f(v['p'])} | {f(v['ci'][0])} to {f(v['ci'][1])} | "
                     f"{f(v['median_min_sep'])} |")
        L.append("")
    g = R["global"]
    L += ["## Global (Latin hypercube)", "",
          f"{g['n_samples']} samples, {g['trials_per_sample']} trials each, log uniform over the "
          "ranges. Spearman rank correlation with the median minimum separation (negative means "
          "that raising the parameter reduces separation). p values are not corrected for "
          "multiple comparisons.", "",
          "| Parameter | rho (median sep) | p | rho (P viol) | p |", "|---|---|---|---|---|"]
    for p, c in sorted(g["spearman"].items(), key=lambda kv: -abs(kv[1]["rho_median_min_sep"])):
        L.append(f"| {p} | {c['rho_median_min_sep']:+.2f} | {c['p_value_median']:.3g} | "
                 f"{c['rho_p_violation']:+.2f} | {c['p_value_pviol']:.3g} |")
    L += ["", "## What survives and what does not", ""]
    st = R["ranking_by_swing"].get("stressed_spacing_4.0", [])
    be = R["ranking_by_swing"].get("benign_spacing_5.0", [])
    if st:
        L.append("Top three by swing in median minimum separation, stressed baseline: "
                 + ", ".join(r["param"] for r in st[:3]) + ".")
    if be:
        L.append("Top three, benign baseline: " + ", ".join(r["param"] for r in be[:3]) + ".")
    # a range test of the wind claim
    sw = R["oat"].get("stressed_spacing_4.0", {}).get("params", {})
    if sw:
        worst = max(sw, key=lambda p: abs(sw[p]["high"]["p"] - sw[p]["low"]["p"]))
        L.append(f"Largest change in P(violation) across a parameter's range: {worst} "
                 f"({f(sw[worst]['low']['p'])} to {f(sw[worst]['high']['p'])}).")
        L.append("")
        flips = [p for p, v in sw.items()
                 if (v["low"]["p"] < 0.05) != (v["high"]["p"] < 0.05)]
        L.append("Parameters whose range moves P(violation) across 0.05 (so any statement that "
                 "the configuration is safe or unsafe depends on them): "
                 + (", ".join(flips) if flips else "none") + ".")
    L += ["", "Conclusions that do not survive the tested ranges are listed above and must not be "
          "quoted without the parameter range. Any single wind speed threshold is a function of "
          "the tracking loop, GNSS and spacing assumptions, and has no meaning on its own.", ""]
    Path("docs/SENSITIVITY.md").write_text("\n".join(L))


if __name__ == "__main__":
    main()
