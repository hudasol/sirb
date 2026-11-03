import json

import pytest

from sirb.disturbance import StressConfig
from sirb.pack import build_pack, check_hashes
from sirb.spec import new_show


def _show():
    grid = {"type": "launch_grid", "spacing_m": 3.2, "center": [0.0, -40.0, 0.0]}
    return new_show("pack test", 9, [
        {"name": "launch", "shape": grid, "hold_s": 1},
        {"name": "orb", "shape": {"type": "sphere", "spacing_m": 3.2, "center": [0, 0, 40]}, "hold_s": 2},
        {"name": "land", "shape": grid, "hold_s": 1}], seed=3)


def test_pack_contents_hashes_and_summary(tmp_path):
    cfg = StressConfig(dt=0.2, gnss_bias_sigma=0.1, gnss_deg_fraction=0.2)
    r = build_pack(_show(), tmp_path / "p", cfg, trials=10, do_repair=False, with_band=False)
    names = set(r["files"])
    assert {"show.json", "export.json", "verification.json", "diagnosis.json", "repair_log.json",
            "stress_summary.json", "limitations.txt", "summary.html"} <= names
    assert check_hashes(tmp_path / "p") == []
    html = (tmp_path / "p" / "summary.html").read_text(encoding="utf-8")
    assert 'dir="rtl"' in html and "Not flight certified" in html and "<script" not in html
    d = json.loads((tmp_path / "p" / "diagnosis.json").read_text())
    assert d["environment"]["numpy"]
    (tmp_path / "p" / "show.json").write_text("{}")
    assert check_hashes(tmp_path / "p") == ["show.json"]


def test_signature_roundtrip(tmp_path):
    pytest.importorskip("cryptography")
    from sirb.pack import keygen, sign, verify_signature, write_hashes
    p = tmp_path / "p"
    p.mkdir()
    (p / "a.txt").write_text("x")
    write_hashes(p)
    pub = keygen(tmp_path / "k.pem")
    sign(p, tmp_path / "k.pem")
    assert verify_signature(p, pub)
    (p / "SHA256SUMS").write_text("tampered\n")
    assert not verify_signature(p, pub)
