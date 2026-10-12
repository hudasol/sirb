"""Run axe-core on the static viewer pages and write results/axe.json.

axe-core 4.14.0 (MPL-2.0) is kept in scripts/vendor/axe-core/ and injected only by this script; it is
not part of the shipped site. Pages: index.html (EN and AR, dark and light, with a show loaded and a
scenario selected) and methods.html (EN and AR, dark and light).

    python scripts/axe_check.py [--label after] [--out results/axe.json]

The output file keeps one entry per label ("before", "after"), so re-running with another label adds to it.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from _viewer_pw import browser, prefs_script, serve, wait_loaded

AXE = Path(__file__).resolve().parent / "vendor" / "axe-core" / "axe.min.js"
IMPACTS = ["critical", "serious", "moderate", "minor", None]


def run_page(b, base, page_name, lang, theme):
    ctx = b.new_context(viewport={"width": 1440, "height": 900}, bypass_csp=True)
    ctx.add_init_script(prefs_script(lang, theme))
    page = ctx.new_page()
    page.goto(f"{base}/{page_name}")
    if page_name == "index.html":
        wait_loaded(page)
    else:
        page.wait_for_selector("h1")
    page.add_script_tag(content=AXE.read_text(encoding="utf-8"))
    res = page.evaluate("async () => axe.run(document, {resultTypes: ['violations','incomplete']})")
    ctx.close()

    def slim(items):
        return [{"id": r["id"], "impact": r.get("impact"), "help": r["help"],
                 "nodes": [{"target": n["target"], "summary": (n.get("failureSummary") or "")[:300]}
                           for n in r["nodes"]]} for r in items]
    return {"page": page_name, "lang": lang, "theme": theme, "axe_version": res["testEngine"]["version"],
            "violations": slim(res["violations"]), "incomplete": slim(res["incomplete"])}


def tally(items):
    by = {str(k): 0 for k in IMPACTS}
    nodes = 0
    for r in items:
        by[str(r["impact"])] += 1
        nodes += len(r["nodes"])
    return {"rules": len(items), "nodes": nodes, "by_impact": by}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="after")
    ap.add_argument("--out", default="results/axe.json")
    args = ap.parse_args()
    runs = []
    with serve() as base, browser() as b:
        for page_name in ("index.html", "methods.html"):
            for lang in ("en", "ar"):
                for theme in ("dark", "light"):
                    r = run_page(b, base, page_name, lang, theme)
                    r["violation_counts"] = tally(r["violations"])
                    r["incomplete_counts"] = tally(r["incomplete"])
                    runs.append(r)
                    print(page_name, lang, theme, "violations", r["violation_counts"]["rules"],
                          "(nodes", r["violation_counts"]["nodes"], ") incomplete",
                          r["incomplete_counts"]["rules"], flush=True)
    summary = {"violation_rules": sum(r["violation_counts"]["rules"] for r in runs),
               "violation_nodes": sum(r["violation_counts"]["nodes"] for r in runs),
               "incomplete_rules": sum(r["incomplete_counts"]["rules"] for r in runs),
               "incomplete_nodes": sum(r["incomplete_counts"]["nodes"] for r in runs),
               "violations_by_impact": {k: sum(r["violation_counts"]["by_impact"][k] for r in runs)
                                        for k in map(str, IMPACTS)},
               "incomplete_by_impact": {k: sum(r["incomplete_counts"]["by_impact"][k] for r in runs)
                                        for k in map(str, IMPACTS)}}
    out = Path(args.out)
    data = json.loads(out.read_text()) if out.exists() else {}
    data["axe_core"] = {"version": runs[0]["axe_version"], "license": "MPL-2.0",
                        "file": "scripts/vendor/axe-core/axe.min.js"}
    data[args.label] = {"summary": summary, "runs": runs}
    out.write_text(json.dumps(data, indent=1, ensure_ascii=False))
    print(args.label, summary)


if __name__ == "__main__":
    main()
