import json
from pathlib import Path

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from sirb.shapes import generate_points, min_nn_distance
from sirb.spec import load_show, validate_show
from sirb.trajectory import Fleet, Segment, Trajectory

SHAPES = [
    {"type": "sphere"}, {"type": "ring"}, {"type": "helix"}, {"type": "torus"},
    {"type": "galaxy"}, {"type": "wall"}, {"type": "skyline"}, {"type": "supershape"},
    {"type": "text", "text": "SIRB"},
]


@pytest.mark.parametrize("shape", SHAPES)
@pytest.mark.parametrize("n", [20, 77])
def test_shape_count_and_spacing(shape, n):
    pts = generate_points({**shape, "spacing_m": 3.0}, n, seed=3)
    assert pts.shape == (n, 3)
    assert min_nn_distance(pts) == pytest.approx(3.0, rel=1e-6)


def test_deterministic():
    a = generate_points({"type": "galaxy", "spacing_m": 3.0}, 60, seed=5)
    b = generate_points({"type": "galaxy", "spacing_m": 3.0}, 60, seed=5)
    assert np.array_equal(a, b)


def test_unknown_letter_rejected():
    with pytest.raises(ValueError):
        generate_points({"type": "text", "text": "Q", "spacing_m": 3.0}, 30)


def test_size_and_spacing_exclusive():
    with pytest.raises(ValueError):
        generate_points({"type": "sphere", "size": 5, "spacing_m": 3.0}, 30)


def test_demo_shows_valid():
    files = sorted(Path("shows").glob("[!g]*.json"))
    assert len(files) >= 4
    for f in files:
        show = json.loads(f.read_text())
        assert validate_show(show) == [], f.name
        load_show(f)


def test_validator_catches_problems():
    base = json.loads(sorted(Path("shows").glob("[!g]*.json"))[0].read_text())
    bad = json.loads(json.dumps(base))
    bad["limits"]["geofence"]["y"][0] = -240.0
    assert any("crowd" in e for e in validate_show(bad))
    bad = json.loads(json.dumps(base))
    bad["stages"][1]["shape"]["spacing_m"] = 1.0
    assert any("spacing" in e for e in validate_show(bad))
    bad = json.loads(json.dumps(base))
    del bad["limits"]
    assert validate_show(bad)
    bad = json.loads(json.dumps(base))
    bad["stages"][1]["shape"]["type"] = "logo"
    assert any("unknown shape" in e for e in validate_show(bad))


def test_fleet_roundtrip_and_hash():
    seg = Segment(0.0, 2.0, np.array([[0, 0, 0], [1, 0, 0], [0, 0.5, 0]], float))
    fleet = Fleet([Trajectory([seg])])
    again = Fleet.from_dict(json.loads(json.dumps(fleet.to_dict())))
    assert again.content_hash() == fleet.content_hash()
    assert np.allclose(again.positions(1.0), fleet.positions(1.0))
    assert np.allclose(fleet.positions(1.0), [[1.0, 0.5, 0.0]])


@given(st.integers(2, 120), st.integers(0, 50))
@settings(max_examples=25, deadline=None)
def test_launch_grid_spacing_property(n, seed):
    pts = generate_points({"type": "launch_grid", "spacing_m": 3.0}, n, seed)
    assert pts.shape == (n, 3)
    assert min_nn_distance(pts) >= 3.0 - 1e-9
