"""viewer/results.html is generated from results/*.json and must stay in step with them."""

import hashlib
import importlib.util
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _gen():
    spec = importlib.util.spec_from_file_location("make_results_page", ROOT / "scripts" / "make_results_page.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_committed_page_matches_generator():
    mod = _gen()
    committed = (ROOT / "viewer" / "results.html").read_bytes().replace(b"\r\n", b"\n").decode("utf-8")
    assert committed == mod.page().replace("\r\n", "\n"), "run python scripts/make_results_page.py"


def test_every_row_names_a_result_file_with_its_current_hash():
    text = (ROOT / "viewer" / "results.html").read_text(encoding="utf-8")
    pairs = re.findall(r"results/([A-Za-z0-9_]+\.json)\"><code>[^<]+</code></a> <span class=\"meta\">sha256 ([0-9a-f]{12})", text)
    assert len(pairs) >= 10
    for name, sha in pairs:
        raw = (ROOT / "results" / name).read_bytes().replace(b"\r\n", b"\n")
        assert hashlib.sha256(raw).hexdigest()[:12] == sha, name


def test_page_states_the_limitations_and_wording():
    text = (ROOT / "viewer" / "results.html").read_text(encoding="utf-8")
    assert 'id="limitations"' in text
    assert "nominal trajectory passed the specified separation check" in text
