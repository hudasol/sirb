"""Frozen headline numbers (2026-11-03, before the assurance loop work packages).

If any of these changes, a change to the planners, the exact verifier, the second verifier or an
existing result file has altered an existing number. Revert that change and record the reason in
docs/DECISIONS_OVERNIGHT.md rather than editing the constants here.
"""

import json
from pathlib import Path

import pytest

from sirb.planner.baseline import plan_baseline
from sirb.spec import load_show
from sirb.verify import verify

# exact minimum centre separation of the baseline plan, metres (committed viewer data, 6 decimals)
SHOW_MIN_SEP = {"gallery_120": 2.155806, "oryx_500": 2.521361, "torus_300": 2.195874,
                "sirb_text_100": 2.188611}

DIFFERENTIAL = {"random_fleets": 8000, "adversarial_fleets": 2750, "larger_fleets_12_drones": 100,
                "hundred_drone_fleets": 6, "total_fleets": 10856, "verdict_checks": 13100,
                "n_disagreement_records": 30}
DIFFERENTIAL_VERDICTS = {"PASS": 4990, "FAIL": 5558, "UNDECIDED": 2552,
                         "pass_with_exact_below_threshold": 0}

# phase 6 mutation results: (killed, survived, score)
MUTATION = {"sirb/assign.py": (292, 22, 0.9299), "sirb/planner/baseline.py": (185, 3, 0.984),
            "sirb/planner/common.py": (187, 9, 0.9541), "sirb/planner/prioritized.py": (565, 110, 0.8284),
            "sirb/planner/waves.py": (405, 45, 0.9), "sirb/verify.py": (2214, 156, 0.9291),
            "sirb/verify2.py": (942, 39, 0.9534)}


def test_viewer_index_min_separations_are_frozen():
    idx = json.loads(Path("viewer/data/index.json").read_text(encoding="utf-8"))
    got = {s["id"]: s["min_separation_m"] for s in idx["shows"]}
    assert got == SHOW_MIN_SEP


@pytest.mark.parametrize("name", ["sirb_text_100", "gallery_120"])
def test_exact_separation_of_demo_plans_is_frozen(name):
    show = load_show(f"shows/{name}.json")
    fleet, _ = plan_baseline(show)
    rep = verify(fleet, show["limits"])
    assert rep.ok
    assert round(rep.min_separation, 6) == SHOW_MIN_SEP[name]
    idx = json.loads(Path("viewer/data/index.json").read_text(encoding="utf-8"))
    assert fleet.content_hash() == next(s["fleet_sha256"] for s in idx["shows"] if s["id"] == name)


def test_differential_counts_are_frozen():
    d = json.loads(Path("results/differential.json").read_text(encoding="utf-8"))
    for k, v in DIFFERENTIAL.items():
        assert d["counts"][k] == v, k
    for k, v in DIFFERENTIAL_VERDICTS.items():
        assert d["verdicts"][k] == v, k


def test_mutation_scores_are_frozen():
    m = json.loads(Path("results/mutation.json").read_text(encoding="utf-8"))["phase6"]
    for f, (killed, survived, score) in MUTATION.items():
        assert (m[f]["killed"], m[f]["survived"], m[f]["score"]) == (killed, survived, score), f
