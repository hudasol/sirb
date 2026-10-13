"""Summarise mutmut results into results/mutation.json (docs/MUTATION.md is generated from it).

Reads the per-file ``mutants/sirb/**/*.meta`` files that mutmut keeps (exit code per mutant) and
maps them to mutmut's own status names (``mutmut.stats.status_by_exit_code``). Nothing is typed by
hand.

    python scripts/mutation_summary.py --label before            # counts only
    python scripts/mutation_summary.py --label after --diffs     # also record each survivor's change
    python scripts/mutation_summary.py --label after --markdown  # print the tables used in the docs

Formula (stated in docs/MUTATION.md):

    score          = killed / (total - skipped)
    score_detected = (killed + timeout) / (total - skipped)

``total`` counts every generated mutant (killed, survived, timeout, suspicious, no tests, not
checked, skipped). A mutant in code the selected tests never execute ("no tests") counts against
the score; a timeout is reported separately and counted as detected only in ``score_detected``.
"""

from __future__ import annotations

import argparse
import ast
import difflib
import json
import re
from pathlib import Path

from mutmut.stats import status_by_exit_code

ROOT = Path(__file__).resolve().parents[1]
STATUSES = ["killed", "survived", "timeout", "suspicious", "no tests", "skipped", "not checked"]


def collect(mutants_dir: Path) -> dict:
    files: dict = {}
    for meta in sorted((mutants_dir / "sirb").rglob("*.meta")):
        rel = "sirb/" + str(meta.relative_to(mutants_dir / "sirb"))[: -len(".meta")]
        data = json.loads(meta.read_text())
        counts = {s: 0 for s in STATUSES}
        names: dict = {s: [] for s in STATUSES}
        for name, code in data["exit_code_by_key"].items():
            st = status_by_exit_code[code]
            if st not in counts:  # e.g. segfault, interrupted: fold into suspicious
                st = "suspicious"
            counts[st] += 1
            names[st].append(name)
        total = sum(counts.values())
        denom = total - counts["skipped"]
        files[rel] = {
            "total": total,
            **{s.replace(" ", "_"): counts[s] for s in STATUSES},
            "score": round(counts["killed"] / denom, 4) if denom else None,
            "score_detected": round((counts["killed"] + counts["timeout"]) / denom, 4)
            if denom else None,
            "_survivor_names": sorted(names["survived"]),
            "_other_names": sorted(names["timeout"] + names["suspicious"] + names["no tests"]
                                   + names["not checked"]),
        }
    return files


_SRC_CACHE: dict = {}


def _function_lines(rel: str):
    """Lines and function line spans of the mutated copy of ``rel`` (mutmut writes every mutant of
    a function into the same file as ``x_<f>__mutmut_<n>``, next to ``x_<f>__mutmut_orig``)."""
    if rel not in _SRC_CACHE:
        text = (ROOT / "mutants" / rel).read_text()
        spans = {n.name: (n.lineno, n.end_lineno) for n in ast.walk(ast.parse(text))
                 if isinstance(n, ast.FunctionDef)}
        _SRC_CACHE[rel] = (text.splitlines(), spans)
    return _SRC_CACHE[rel]


def diff_line(rel: str, name: str) -> str:
    lines, spans = _function_lines(rel)
    func, _, num = name.rsplit(".", 1)[1].rpartition("__mutmut_")
    take = lambda fn: lines[spans[fn][0] - 1: spans[fn][1]]  # noqa: E731
    d = difflib.unified_diff(take(func + "__mutmut_orig"), take(f"{func}__mutmut_{num}"),
                             lineterm="", n=0)
    ch = [ln for ln in d if ln[:1] in "+-" and not ln.startswith(("+++", "---"))]
    ch = [ln for ln in ch if "__mutmut_" not in ln]
    minus = [ln[1:].strip() for ln in ch if ln[0] == "-"]
    plus = [ln[1:].strip() for ln in ch if ln[0] == "+"]
    return f"{func[2:]}: {' / '.join(minus)}  ->  {' / '.join(plus) or '(removed)'}"



# (function, regex on the change text, class, reason). Hand-written after reading the code; the
# justification is argued in docs/MUTATION.md, not proven.
OVERRIDES = [
    ("_build_pieces", r"last_pos = None", "equivalent", "overwritten by the first segment of the loop before any use"),
    ("_build_pieces", r"t0r\.append\(1\.0\)", "equivalent", "constant (hold) piece: the time origin does not enter a degree-0 polynomial"),
    ("_build_pieces", r"default=2", "equivalent", "only reached with no pieces, which pair_minima handles before using K"),
    ("_horner_rows", r"- coef\[", "equivalent", "negates the whole polynomial; only its norm is used"),
    ("_build_pieces", r"range\(0, len\(real\), \)", "equivalent", "chunk size only changes memory use"),
    ("_build_pieces", r"hi\[idx\] \+ P\.lo", "no-effect", "box over a wider span is a superset; prefilter is only less selective"),
    ("_candidate_pairs", r"- 1e-9\) \*\* 2", "tolerance", "prefilter radius slack of 1e-9 m"),
    ("_candidate_pairs", r"j_end <= i - 1", "equivalent", "ends[i] >= i + 1 always (hi > lo), so the branch is the same"),
    ("_candidate_pairs", r"sel = np\.einsum", "no-effect", "same-drone pairs are discarded later by the pair key"),
    ("_candidate_pairs", r"np\.zeros\(", "equivalent", "empty result; dtype of an empty index array is irrelevant"),
    ("_critical_points", r"ev = ", "equivalent", "defensive LinAlgError fallback; eigvals does not raise for the finite input built here"),
    ("_critical_points", r"^", "deep-numerics", "root-finding details (degree selection, Newton polish, masks); the 9-point grid and direct evaluation make results insensitive on the test polynomials"),
    ("_extremes", r"np\.where\(None|, 1\.0, s\)", "equivalent", "an invalid slot evaluated at s=0 (or 1) is a genuine endpoint candidate"),
    ("_extremes", r"^", "deep-numerics", "extremum bookkeeping"),
    ("_refine_minimum", r"^", "deep-numerics", "golden-section refinement only lowers a minimum towards the true one; needs a near-tangent known-answer case"),
    ("_shift", r"astype\(None\)|, None\)\.astype", "equivalent", "dtype/fill of the integer exponent table; masked entries are zeroed afterwards"),
    ("_solve_pairs", r"slice\(", "equivalent", "slice(None,3), slice(0,None), slice(3) select the same three columns"),
    ("_solve_pairs", r"np\.where\(None, 0\.0, sc\)|bad = |np\.where\(None, np\.inf", "equivalent", "invalid-root masking; invalid slots are replaced by candidates already present"),
    ("_solve_pairs", r"dist|pt = |tp = |pa = |pb = |b \+ a", "equivalent",
     "point evaluation of unbounded/degenerate overlaps; dominated by the adjacent finite pair for continuous trajectories (argued, not proven)"),
    ("_solve_pairs", r"^", "deep-numerics", "per-pair minimisation details"),
    ("_sq_poly", r"\+ 1\)", "equivalent", "one extra all-zero coefficient column"),
    ("_structure", r"bad\.add\(None\)|append\(None\)", "equivalent", "drone is excluded anyway because it has no usable segment"),
    ("pair_minima", r"key = ", "equivalent", "all variants of the pair key remain injective for pieces of drones <= max index"),
    ("pair_minima", r"refine_below", "no-effect", "refining every pair instead of only those below the radius changes cost, not the reported minima below the radius"),
    ("verify", r"abs\(v\.value \+ v\.limit\)", "open", "order of violations under the cap is by |value - limit|; only one cap test"),
    ("verify", r"pieces = None|_pieces=None|pair_minima\(None|_pieces=pieces\)$|, \)$", "equivalent", "pair_minima rebuilds the same pieces from the fleet when none are passed"),
]

def classify(path: str, change: str) -> tuple[str, str]:
    """Rule-based triage of one survivor from its textual change (first matching rule wins).

    The classes are those of docs/MUTATION.md. The rules only label what the change text itself
    shows; anything they cannot place is ``open`` and is not claimed to be equivalent.
    """
    func, _, rest = change.partition(": ")
    minus, _, plus = rest.partition("  ->  ")
    if "errstate" in minus:
        return "no-effect", "floating-point warning filter; changes warnings, not values"
    if minus.strip() == "" or minus.strip().startswith(('"""', "#")):
        return "equivalent", "docstring or comment removed"
    if path.endswith("verify.py"):
        for fn, rx, cls, why in OVERRIDES:
            if rx != "^" and func == fn and re.search(rx, change.partition(": ")[2]):
                return cls, why
    if minus.lower() == plus.lower() and minus != plus:
        return "equivalent", "case-insensitive argument (scipy/numpy/einsum/float parsing)"
    if re.search(r"\bsmax\b|float\(\"NAN\"\)", plus) and "NAN" in plus:
        return "equivalent", "float('NAN') parses to the same NaN"
    for kw, why in (("dtype", "default dtype of the values involved is the same"),
                    ("kind", "sort kind only matters for exact ties"),
                    ("side", "searchsorted default side is 'left'"),
                    ("nan", "nan_to_num default is 0.0")):
        if f"{kw}=" in minus and f"{kw}=" not in plus and f"{kw}=" not in plus.replace(kw, ""):
            return "equivalent", why + " (argument removed)"
        if f"{kw}=" in minus and re.search(rf"{kw}=None", plus):
            return "equivalent", why + " (argument set to None = default)"
    if "dtype" in minus and re.search(r"dtype=None|, \)", plus):
        return "equivalent", "default dtype of the values involved is the same"
    if "or True" in plus or "and False" in plus:
        return "equivalent", "prefilter/cap switch only widens an optional pre-selection"
    mm, pp = minus.replace("<=", "<").replace(">=", ">"), plus.replace("<=", "<").replace(">=", ">")
    if mm == pp and minus != plus:
        return "tolerance", "strict vs non-strict comparison; differs only when the two sides are exactly equal"
    nums = re.compile(r"\d+\.?\d*(?:e[-+]?\d+)?", re.I)
    if nums.sub("#", minus) == nums.sub("#", plus) and minus != plus:
        return "constant", "numeric constant changed; no input in the tests distinguishes it"
    if "XX" in plus or "detail" in plus or re.search(r"f\"", minus):
        return "message", "message or report text"
    if path.endswith("verify.py"):
        for fn, rx, cls, why in OVERRIDES:
            if rx == "^" and func == fn:
                return cls, why
    if path.endswith("prioritized.py"):
        return "heuristic", "search-heuristic or schedule parameter; any variant still yields a plan the verifier judges"
    if path.endswith(("waves.py", "baseline.py", "common.py", "assign.py")):
        return "open", "planner internals not pinned by a known-answer test"
    return "open", "not shown to be equivalent; no test distinguishes it"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", required=True, help="e.g. before or after")
    ap.add_argument("--out", default=str(ROOT / "results" / "mutation.json"))
    ap.add_argument("--diffs", action="store_true", help="record the change made by each survivor")
    ap.add_argument("--only", nargs="*", help="restrict to these source files (e.g. sirb/verify.py)")
    ap.add_argument("--markdown", action="store_true")
    ap.add_argument("--tables", action="store_true",
                    help="print the before/after table and the grouped survivor table for docs/MUTATION.md")
    args = ap.parse_args()

    files = collect(ROOT / "mutants")
    if args.only:
        files = {k: v for k, v in files.items() if k in args.only}
    out = Path(args.out)
    doc = json.loads(out.read_text()) if out.exists() else {}
    doc.setdefault("formula", "score = killed / (total - skipped); score_detected = "
                              "(killed + timeout) / (total - skipped)")
    doc.setdefault("tool", "mutmut 3.8.0")
    section = doc.setdefault(args.label, {})
    for path, rec in files.items():
        surv = rec.pop("_survivor_names")
        other = rec.pop("_other_names")
        rec["survivors"] = []
        for n in surv:
            ch = diff_line(path, n) if args.diffs else None
            item = {"mutant": n, "change": ch}
            if ch is not None:
                item["class"], item["reason"] = classify(path, ch)
            rec["survivors"].append(item)
        rec["other_undetected"] = other
        section[path] = rec
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(doc, indent=1, sort_keys=True) + "\n")
    if args.tables:
        print(tables(doc))
    if args.markdown:
        print("| file | total | killed | survived | timeout | suspicious | no tests | score |")
        print("|---|---|---|---|---|---|---|---|")
        for path, rec in sorted(section.items()):
            sc = "n/a" if rec["score"] is None else f"{100 * rec['score']:.1f}%"
            print(f"| {path} | {rec['total']} | {rec['killed']} | {rec['survived']} | "
                  f"{rec['timeout']} | {rec['suspicious']} | {rec['no_tests']} | {sc} |")


def tables(doc: dict) -> str:
    """Markdown for docs/MUTATION.md, generated only from results/mutation.json."""
    out = ["| file | mutants | before: killed | before: score | after: killed | after: survived | "
           "after: timeout | after: score |", "|---|---|---|---|---|---|---|---|"]
    pc = lambda r: "n/a" if r["score"] is None else f"{100 * r['score']:.1f}%"  # noqa: E731
    for f, a in sorted(doc["after"].items()):
        b = doc["before"][f]
        out.append(f"| {f} | {a['total']} | {b['killed']} | {pc(b)} | {a['killed']} | "
                   f"{a['survived']} | {a['timeout']} | {pc(a)} |")
    out += ["", "Survivors after the new tests, grouped by function and class (`mutant numbers` are the "
            "`__mutmut_<n>` suffixes in `results/mutation.json`).", "",
            "| file | function | class | n | mutants | reason |", "|---|---|---|---|---|---|"]
    for f, a in sorted(doc["after"].items()):
        groups: dict = {}
        for sv in a["survivors"]:
            fn = sv["change"].split(":")[0] if sv.get("change") else "?"
            groups.setdefault((fn, sv["class"], sv["reason"]), []).append(
                sv["mutant"].rsplit("__mutmut_", 1)[1])
        for (fn, cl, why), nums in sorted(groups.items()):
            nums = sorted(nums, key=int)
            shown = ", ".join(nums) if len(nums) <= 12 else ", ".join(nums[:12]) + ", ..."
            out.append(f"| {f.removeprefix('sirb/')} | {fn} | {cl} | {len(nums)} | {shown} | {why} |")
    return "\n".join(out)


if __name__ == "__main__":
    main()
