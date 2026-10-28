"""The stage windows and evidence shipped to the viewer must be consistent with the show data."""

import json
from pathlib import Path

IDX = json.loads(Path("viewer/data/index.json").read_text(encoding="utf-8"))


def test_stage_windows_are_ordered_and_cover_the_show():
    for sh in IDX["shows"]:
        st = sh["stages"]
        show = json.loads(Path(f"shows/{sh['id']}.json").read_text(encoding="utf-8"))
        assert [s["id"] for s in st] == [s["name"] for s in show["stages"]]
        assert st[0]["t_start"] == 0.0
        for a, b in zip(st, st[1:]):
            assert a["t_start"] <= a["t_arrive"] <= a["t_end"] <= b["t_start"] + 1e-9
        assert abs(st[-1]["t_end"] - sh["duration_s"]) < 1e-6
        assert sh["blurb_en"] and sh["blurb_ar"] and all(s["name_en"] and s["name_ar"] for s in st)


def test_evidence_matches_the_result_files():
    ev = IDX["evidence"]
    base = json.loads(Path("results/baselines.json").read_text())["sampled_false_pass"]
    assert ev["crossings"]["trials"] == base["trials"]
    for k, v in base["false_pass_rate"].items():
        assert abs(ev["crossings"]["false_pass_rate"][k] - v) < 1e-6
    assert ev["razor"]["exact_wrong"] == 0


def test_gallery_is_the_default_show():
    assert IDX["shows"][0]["id"] == "gallery_120"
