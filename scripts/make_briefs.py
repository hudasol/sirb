"""Generate the reviewer pack: docs/BRIEF_TII_ATRC.md, docs/BRIEF_EDGE.md, docs/BRIEF_ADEO.md and
docs/SUMMARY_AR.md. Every number is read from results/*.json or viewer/data/index.json.

The briefs are proposals, not achievements. They describe Sirb, not the organisations: no claim is
made about what any organisation does, wants or has agreed to, and no partnership or endorsement
exists. Audience specific questions are the owner's framing and are marked as such.

    python scripts/make_briefs.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from make_results_doc import load  # noqa: E402

DOCS = Path("docs")


def f(x, n=3):
    return "n/a" if x is None else f"{x:.{n}f}"


def facts() -> dict:
    """The numbers the briefs quote, each with the file it comes from."""
    F: dict = {}
    idx = json.loads(Path("viewer/data/index.json").read_text())
    F["shows"] = [(s["id"], s["n"], s["min_separation_m"], s["d_min_m"], s["verdict"])
                  for s in idx["shows"]]
    d = load("differential.json")
    if d:
        F["diff_fleets"] = d["counts"]["total_fleets"]
        F["diff_checks"] = d["counts"]["verdict_checks"]
        F["diff_max_l_minus_e"] = d["max_discrepancies"]["max_l_minus_e"]
    m = load("mutation.json")
    if m and "sirb/verify.py" in m["after"]:
        F["mut_verify"] = m["after"]["sirb/verify.py"]["score"]
    g = load("gnss_faa_derivation.json")
    if g:
        F["faa_sigma"] = g["statistics"]["bias_sigma_median_m"]
        F["faa_sites"] = g["statistics"]["n_sites"]
    s1 = load("sensitivity_faa.json")
    if s1:
        F["faa_p"] = {k: v["baseline"]["p"] for k, v in s1["oat"].items()}
    s0 = load("sensitivity.json")
    if s0:
        F["assumed_p"] = {k: v["baseline"]["p"] for k, v in s0["oat"].items()}
    mg = load("margin.json")
    if mg:
        F["margin"] = {(r["case"]["n"], r["case"].get("bias_sigma", "faa")): r for r in mg["rows"]}
        F["margin_trials"] = mg["trials"]
    gr = load("ground_robots.json")
    if gr:
        rows = [r for r in gr["rows"] if r["planner"] == "grid"]
        F["grid_ok"] = sum(1 for r in rows if r.get("verified_ok"))
        F["grid_runs"] = len(rows)
    return F


def layers(F) -> list[str]:
    shows = "; ".join(f"`{i}` ({n} drones): {f(m)} m against {d} m, {v}" for i, n, m, d, v in F["shows"])
    L = ["## What is guaranteed, simulated and only planned", "",
         "**Guaranteed under the model** (drones are points on the polynomial paths as written, limits "
         "as declared in the show file): the exact verifier's minimum separation and limit checks. "
         f"Demo shows: {shows} (`viewer/data/index.json`)."]
    if "diff_fleets" in F:
        L.append(f"Cross check: a second verifier with a different method agreed on {F['diff_fleets']} "
                 f"fleets and {F['diff_checks']} verdict checks; its lower bound never exceeded the exact "
                 f"value (largest excess {F['diff_max_l_minus_e']:.1e} m) (`results/differential.json`). "
                 "This is evidence, not a proof, and both verifiers have the same author.")
    L += ["",
          "**Simulated** (assumed models, no flight data): every probability of a separation "
          "violation under wind, GNSS error, tracking error and failures (`docs/METHODS.md`)."]
    if "faa_p" in F and "assumed_p" in F:
        L.append("With a GNSS bias magnitude derived from public FAA accuracy statistics "
                 f"({f(F['faa_sigma'])} m per axis, median of {F['faa_sites']} sites, applied as an "
                 "independent error per degraded drone, which overstates relative error): P(violation) "
                 + "; ".join(f"{k} {f(F['assumed_p'][k])} assumed versus {f(v)} sourced"
                             for k, v in F["faa_p"].items()) + " (`docs/GNSS_SOURCED.md`).")
    L += ["", "**Only planned**: any flight, indoor or outdoor; any hardware; any regulatory work. "
          "Nothing has been flown. Sirb has no flight control, radio or MAVLink code.", ""]
    return L


INDOOR = ["## What funded indoor validation with borrowed hardware would test", "",
          "Proposed, not started (`docs/PROPOSAL_SEED.md`). Indoors, netted, with a host laboratory "
          "that owns the safety case and flies its own aircraft with its own tools; Sirb supplies "
          "plan files only.", "",
          "1. Real tracking error of small multirotors following Sirb polynomials, measured by motion "
          "capture, against the simulated tracking error.",
          "2. Whether the verified minimum separation plus the measured tracking error predicts the "
          "measured minimum separation.",
          "3. Identification of the assumed tracking parameters from logs, and the change in simulated "
          "risk when they replace the assumptions.",
          "4. Motion capture is not GNSS: GNSS error models would stay untested indoors.", ""]

FOOT = ["## What this brief does not claim", "",
        "No partnership, endorsement, approval or agreement with any organisation, including the one "
        "named in the title. No cost figure. No regulatory compliance and no flight certification. "
        "A real show needs an operator safety case and approvals from the relevant UAE aviation "
        "authorities. Repository: https://github.com/hudasol/sirb (proposal seed).", ""]


def brief(title, audience_note, question, extra, F) -> str:
    L = [f"# {title}", "",
         "**Proposal seed, not an achievement.** Generated by `scripts/make_briefs.py` from result "
         "files. Simulation and planning only; not flight certified.", "",
         f"*Audience framing.* {audience_note}", "",
         "## What Sirb is", "",
         "A simulation first planner for drone light shows: shape sequences become multi drone "
         "trajectories, an exact separation verifier that shares no code with the planners checks "
         "them, and a Monte Carlo simulator stress tests them against modelled wind, GNSS error and "
         "failures. Entertainment and benign coordination only.", ""]
    L += layers(F)
    L += ["## Open research question for this audience", "", question, ""]
    L += extra
    L += INDOOR + FOOT
    return "\n".join(L)


def margin_line(F) -> list[str]:
    if "margin" not in F:
        return []
    out = ["## Planning aid: separation for a stated simulated risk", "",
           "`python -m sirb margin` returns the nominal separation whose simulated Wilson upper bound "
           f"on P(violation) is at most 5% ({F['margin_trials']} trials per scale), and the plan scaled "
           "to it (`docs/MARGIN.md`). Results (simulated, model dependent):", ""]
    for (n, s), r in sorted(F["margin"].items(), key=lambda kv: str(kv[0])):
        lab = "FAA derived file" if s == "faa" else f"bias sigma {s} m"
        out.append(f"* n = {n}, {lab}: {r['status']}, required nominal minimum separation "
                   f"{f(r['required_nominal_min_separation_m'])} m.")
    return out + [""]


def arabic(F) -> str:
    shows = "، ".join(f"{i} ({n} طائرة): {f(m)} م" for i, n, m, _d, _v in F["shows"])
    L = ["# سرب: ملخص باللغة العربية", "",
         "**يحتاج إلى مراجعة من متحدث أصلي (needs native review).** هذا النص مولَّد بواسطة "
         "`scripts/make_briefs.py`، والأرقام مأخوذة من ملفات النتائج.", "",
         "سرب أداة محاكاة وتخطيط لعروض الطائرات المسيّرة الضوئية. تحوّل تسلسلات الأشكال إلى مسارات "
         "لعدد كبير من الطائرات، ويتحقق منها مدقق دقيق للمسافات الفاصلة لا يشارك المخطط أي شيفرة، "
         "ثم تُختبر في محاكاة مونت كارلو تحت رياح وأخطاء ملاحة وأعطال مفترضة.", "",
         "**ما هو مضمون رياضياً ضمن النموذج:** الحد الأدنى الدقيق للمسافة بين الطائرات على المسارات "
         f"كما كُتبت. العروض التجريبية: {shows}.", "",
         "**ما هو محاكى فقط:** جميع احتمالات انتهاك المسافة الفاصلة، بنماذج مفترضة ودون أي بيانات "
         "طيران حقيقية.", "",
         "**ما هو مخطط فقط:** أي طيران داخلي أو خارجي، وأي عتاد، وأي عمل تنظيمي. لم يُنفَّذ أي طيران.",
         "",
         "ليست الأداة معتمدة للطيران، ولا تدّعي الامتثال لأي لوائح، ولا توجد أي شراكة أو مصادقة من "
         "أي جهة. يحتاج أي عرض حقيقي إلى حالة سلامة من المشغّل وإلى موافقات الجهات المختصة بالطيران "
         "في دولة الإمارات.", ""]
    return "\n".join(L)


def main() -> None:
    F = facts()
    grid = []
    if "grid_runs" in F:
        grid = ["## Exploratory: ground robots", "",
                f"A time expanded grid planner produced verified plans in {F['grid_ok']} of "
                f"{F['grid_runs']} simulated floor tasks (`docs/GROUND_ROBOTS.md`, exploratory).", ""]
    briefs = {
        "BRIEF_TII_ATRC.md": brief(
            "Sirb: brief for TII and ATRC reviewers",
            "Written for applied research reviewers. It does not describe TII or ATRC or their "
            "programmes; the question below is the owner's proposal.",
            "Can a separation guarantee be stated for the fleet as flown, not only for the paths as "
            "written? The exact verifier bounds the planned paths; the gap is tracking error and "
            "GNSS error that is correlated between nearby drones. Public statistics found so far give "
            "only the absolute error of one receiver (`docs/GNSS_SOURCED.md`), not the relative error "
            "between two drones that the separation depends on. Measuring that relative error, and "
            "proving a margin rule from it, is the open question.",
            margin_line(F), F),
        "BRIEF_EDGE.md": brief(
            "Sirb: brief for EDGE reviewers",
            "Written for engineering reviewers interested in software assurance. Scope stays "
            "entertainment and benign coordination; nothing here concerns payloads, weapons or "
            "military use, and the brief does not describe EDGE.",
            "Does an independent exact checker, cross checked by a second method and by mutation "
            "testing, give assurance evidence that an engineering review would accept for multi "
            "vehicle coordination software, and what is still missing (independent authorship, "
            "a formal proof of the verifier, tool qualification)? See `docs/VERIFIER.md` and "
            "`docs/MUTATION.md`.",
            grid, F),
        "BRIEF_ADEO.md": brief(
            "Sirb: brief for ADEO reviewers",
            "Written for reviewers who plan or host public shows. The organisation's remit was not "
            "verified from a retrieved source and is not described here; the owner should confirm "
            "this framing.",
            "How much nominal spacing does a show need for a stated simulated risk under local "
            "conditions, and how quickly does that need grow when GNSS degrades? The margin tool "
            "answers this inside the model; whether the model's answer transfers to real shows is the "
            "open question that only measured data can settle.",
            margin_line(F), F),
    }
    for name, text in briefs.items():
        (DOCS / name).write_text(text, encoding="utf-8")
    (DOCS / "SUMMARY_AR.md").write_text(arabic(F), encoding="utf-8")


if __name__ == "__main__":
    main()
