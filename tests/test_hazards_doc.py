"""docs/HAZARDS.md must not drift from the code: every file or symbol named in the
'Covering check' column has to exist, coverage labels must be from the fixed vocabulary, and the
'Not covered' / 'Partial' lists must match the table."""

from __future__ import annotations

import importlib
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
DOC = ROOT / "docs" / "HAZARDS.md"
LEVELS = {"Covered by exact verifier", "Covered by simulation under ASSUMED model",
          "Partially", "Not covered"}
FILE_EXT = (".py", ".md", ".json")


def _rows() -> list[list[str]]:
    rows = []
    for line in DOC.read_text(encoding="utf-8").splitlines():
        if re.match(r"\| H\d+ \|", line):
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            rows.append(cells)
    return rows


def _tokens(cell: str) -> list[str]:
    return re.findall(r"`([^`]+)`", cell)


def _resolve_dotted(tok: str) -> bool:
    parts = tok.split(".")
    for i in range(len(parts), 0, -1):
        try:
            obj = importlib.import_module(".".join(parts[:i]))
        except ImportError:
            continue
        for name in parts[i:]:
            fields = getattr(obj, "__dataclass_fields__", {})
            if name in fields:
                return name == parts[-1]
            if not hasattr(obj, name):
                return False
            obj = getattr(obj, name)
        return True
    return False


def _token_exists(tok: str) -> bool:
    if "::" in tok:
        path, name = tok.split("::", 1)
        f = ROOT / path
        if not f.is_file():
            return False
        text = f.read_text(encoding="utf-8")
        pat = rf"def {re.escape(name)}\b" if name.startswith(("test_", "exp_")) or name == \
            "family_show" else rf"\b{re.escape(name)}\b"
        return re.search(pat, text) is not None
    if tok.endswith(FILE_EXT):
        return (ROOT / tok).is_file()
    return _resolve_dotted(tok)


def test_table_parses():
    rows = _rows()
    assert len(rows) >= 15
    assert all(len(r) == 7 for r in rows), [r[0] for r in rows if len(r) != 7]
    ids = [r[0] for r in rows]
    assert len(set(ids)) == len(ids)


def test_coverage_vocabulary():
    for r in _rows():
        assert r[5] in LEVELS, (r[0], r[5])


def test_every_covering_check_exists():
    missing = []
    for r in _rows():
        toks = _tokens(r[4])
        if r[5] == "Not covered":
            assert not toks and r[4].strip() == "none", f"{r[0]}: not covered rows must say none"
            continue
        assert toks, f"{r[0]}: covered row names no check"
        missing += [(r[0], t) for t in toks if not _token_exists(t)]
    assert not missing, missing


def _listed(section: str) -> set[str]:
    text = DOC.read_text(encoding="utf-8")
    body = text.split(f"## {section}", 1)[1].split("\n## ", 1)[0]
    ids: set[str] = set()
    for line in body.splitlines():
        m = re.match(r"- ((?:H\d+(?:, )?)+):", line)
        if m:
            ids |= set(re.findall(r"H\d+", m.group(1)))
    return ids


@pytest.mark.parametrize("section,level", [("Not covered", "Not covered"),
                                           ("Partial", "Partially")])
def test_lists_match_table(section, level):
    want = {r[0] for r in _rows() if r[5] == level}
    assert _listed(section) == want


def test_checker_rejects_nonexistent():
    assert not _token_exists("sirb.disturbance.no_such_symbol")
    assert not _token_exists("docs/NOPE.md")
    assert not _token_exists("tests/test_stress.py::test_nope")
    assert _token_exists("sirb.disturbance.StressConfig.gnss_bias_sigma")
