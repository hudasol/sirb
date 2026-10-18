"""Check that every number in the reviewer pack and the README result blocks comes from a file.

For each number in the checked text, the script looks for a numeric value in results/**/*.json,
viewer/data/index.json or shows/*.json that rounds to it at the printed precision (also as a
percentage). Numbers inside code spans and URLs are skipped. The check is a necessary condition
only: a match does not prove that the number comes from the right field, and small integers match
easily.
Writes results/doc_number_check.json; exit status 1 if any number is unmatched.

    python scripts/check_doc_numbers.py
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

DOCS = ["docs/BRIEF_TII_ATRC.md", "docs/BRIEF_EDGE.md", "docs/BRIEF_ADEO.md", "docs/SUMMARY_AR.md"]
README_BLOCKS = ("HEADLINES", "ASSURANCE")
SOURCES = sorted(Path("results").rglob("*.json")) + [Path("viewer/data/index.json")] + sorted(
    Path("shows").glob("*.json"))
NUM = re.compile(r"(?<![\w.])(\d+(?:\.\d+)?(?:e[+-]?\d+)?)(?![\w])")


def values(o, out):
    if isinstance(o, bool):
        return
    if isinstance(o, (int, float)):
        out.add(float(o))
    elif isinstance(o, dict):
        for v in o.values():
            values(v, out)
    elif isinstance(o, list):
        for v in o:
            values(v, out)


def matches(tok: str, pool: set) -> bool:
    x = float(tok)
    if "e" in tok:
        mant = tok.split("e")[0]
        nd = len(mant.split(".")[1]) if "." in mant else 0
        return any(float(f"{v:.{nd}e}") == x for v in pool)
    nd = len(tok.split(".")[1]) if "." in tok else 0
    for v in pool:
        if f"{v:.{nd}f}" == tok or f"{100 * v:.{nd}f}" == tok:
            return True
    return x in pool


def strip(text: str) -> str:
    text = re.sub(r"`[^`]*`", " ", text)                 # code spans: paths, commands, ids
    text = re.sub(r"https?://\S+", " ", text)
    return text


def main() -> int:
    pool: set = set()
    for p in SOURCES:
        try:
            values(json.loads(p.read_text(encoding="utf-8")), pool)
        except (json.JSONDecodeError, UnicodeDecodeError):
            continue
    texts = {d: Path(d).read_text(encoding="utf-8") for d in DOCS}
    readme = Path("README.md").read_text(encoding="utf-8")
    for b in README_BLOCKS:
        m = re.search(rf"<!-- {b}:START -->(.*?)<!-- {b}:END -->", readme, re.S)
        texts[f"README.md ({b})"] = m.group(1) if m else ""
    report = {}
    bad = 0
    for name, t in texts.items():
        toks = NUM.findall(strip(t))
        unmatched = sorted({tok for tok in toks if not matches(tok, pool)}, key=float)
        report[name] = {"numbers": len(toks), "unmatched": unmatched}
        bad += len(unmatched)
    Path("results/doc_number_check.json").write_text(json.dumps(report, indent=1) + "\n")
    for k, v in report.items():
        print(k, v["numbers"], "numbers; unmatched:", v["unmatched"])
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
