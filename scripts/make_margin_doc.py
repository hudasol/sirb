"""Generate docs/MARGIN.md from results/margin.json. No hand typed numbers."""

from __future__ import annotations

import json
from pathlib import Path


def f(x, n=3):
    return "n/a" if x is None else f"{x:.{n}f}"


def main() -> None:
    R = json.loads(Path("results/margin.json").read_text())
    rows = R["rows"]
    L = ["# Margin synthesis (planning aid)", "",
         "Generated from `results/margin.json` by `scripts/make_margin_doc.py`. **Simulated, model "
         "dependent planning aid, not a guarantee.** " + R["model_dependence"], "",
         "## Tool", "",
         "`python -m sirb margin SHOW --gnss PARAMS.json [--w20 V] [--target 0.05] [--trials N] "
         "[--out export.json]` scales every formation of the show about its centre, plans each scale "
         "with the baseline planner, checks the plan with the exact verifier (the **guaranteed** layer: "
         "the nominal minimum separation of the plan as written), and estimates P(separation "
         "violation) in the stress simulator (the **simulated** layer). It returns the smallest scale, "
         "to a relative tolerance, whose 95% Wilson upper bound is at most the target, with a guard "
         "against one lucky row, and writes the plan at that scale (method in `sirb/margin.py`). "
         "Applying it to a real show would be the **planned** layer and is not done here.", "",
         "## Study", "",
         f"Show family: {R['note']}. Scenario: {R['scenario']} plus the bias sigma of each row; "
         f"other values are the assumed defaults. {R['trials']} trials per evaluated scale, seed "
         f"{R['seed']}, relative tolerance {R['tol']}, target {R['target']}. With zero violations "
         f"the smallest attainable upper bound is {f(R['wilson_floor'], 4)}.", "",
         "| n | Bias sigma (m) | GNSS file | Status | Required spacing (m) | Required nominal min "
         "separation (m) | Required margin over d_min (m) | Scales evaluated |",
         "|---|---|---|---|---|---|---|---|"]
    for r in rows:
        c = r["case"]
        L.append(f"| {c['n']} | {c.get('bias_sigma', 'from file')} | {c.get('gnss_file', '')} | "
                 f"{r['status']} | {f(r['required_spacing_m'], 2)} | "
                 f"{f(r['required_nominal_min_separation_m'])} | {f(r['required_margin_m'])} | "
                 f"{len(r['evaluations'])} |")
    ck = Path("results/margin_cli_check.json")
    if ck.exists():
        c = json.loads(ck.read_text())
        L += ["", f"Cross check (`scripts/check_margin_cli.py`, `results/margin_cli_check.json`): "
              f"`{c['command']}` on the case {c['case']} gives a report "
              + ("identical to the study row in every compared field."
                 if c["identical"] else f"that differs in {', '.join(c['differing_keys'])}.")]
    by = {(r["case"]["n"], r["case"].get("bias_sigma")): r["required_margin_m"] for r in rows}
    L += ["", "## Scaling against the pre-registered prediction (P11)", ""]
    m5, m10, m0 = by.get((100, 0.5)), by.get((100, 1.0)), by.get((100, 0.0))
    if m5 and m10:
        L.append(f"* Bias sigma 1.0 m versus 0.5 m at n = 100: margin ratio {f(m10 / m5, 2)} "
                 "(predicted between 1.6 and 2.4).")
    if m0 is not None:
        L.append(f"* Bias sigma 0 at n = 100: required margin {f(m0)} m (predicted below 0.3 m).")
    m50, m400 = by.get((50, 0.5)), by.get((400, 0.5))
    if m50 and m400:
        L.append(f"* n = 400 versus n = 50 at bias sigma 0.5 m: margin ratio {f(m400 / m50, 2)} "
                 "(predicted below 1.25).")
    L += ["", "## Limits", "",
          "The answer is conditional on every assumption of the stress model (`docs/METHODS.md` "
          "sections 5 and 9) and on the show family; scaling a formation also changes its size, so the "
          "geofence and altitude band can stop a scale from verifying. The tolerance makes the answer "
          "conservative by at most that fraction of the spacing. The Monte Carlo bound is a statistical "
          "statement about the simulator, not a probability for real flights."]
    Path("docs/MARGIN.md").write_text("\n".join(L) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
