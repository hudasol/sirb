import copy

import pytest

from sirb.cli import main
from sirb.spec import MAX_DRONES, MAX_HOLD_S, MAX_STAGES, load_show, validate_show

SHOW = "shows/sirb_text_100.json"


@pytest.fixture
def show():
    return load_show(SHOW)


def test_too_many_drones(show):
    show["n_drones"] = MAX_DRONES + 1
    assert any("maximum" in e for e in validate_show(show))


def test_too_many_stages(show):
    show["stages"] = show["stages"][:1] * (MAX_STAGES + 1)
    assert any("stages" in e and "maximum" in e for e in validate_show(show))


def test_hold_cap(show):
    s = copy.deepcopy(show)
    s["stages"][0]["hold_s"] = MAX_HOLD_S + 1
    assert any("hold_s" in e for e in validate_show(s, check_targets=False))


@pytest.mark.parametrize("flag", [["--target", "0"], ["--target", "1.5"], ["--trials", "0"],
                                  ["--tol", "0"], ["--w20", "-1"]])
def test_margin_arguments_rejected(flag, capsys):
    rc = main(["margin", SHOW, "--gnss", "shows/gnss_assumed_default.json", *flag])
    assert rc == 1 and "error" in capsys.readouterr().err


def test_bad_json_is_a_clean_error(tmp_path, capsys):
    p = tmp_path / "x.json"
    p.write_text("{not json")
    assert main(["validate", str(p)]) == 1
    assert "error" in capsys.readouterr().err
