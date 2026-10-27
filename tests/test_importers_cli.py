import json

import pytest

from sirb.cli import main
from sirb.importers import fleet_from_csv


def write(tmp_path, text, name="t.csv"):
    p = tmp_path / name
    p.write_text(text)
    return str(p)


CROSS = "drone,t,x,y,z\n0,0,-10,0,5\n0,10,10,0,5\n1,0,10,0.5,5\n1,10,-10,0.5,5\n"


def test_csv_roundtrip_and_verdict(tmp_path, capsys):
    p = write(tmp_path, CROSS)
    f = fleet_from_csv(p)
    assert len(f.trajectories) == 2
    assert main(["verify-external", p, "--d-min", "2"]) == 2
    out = capsys.readouterr().out
    assert "FAIL" in out and "0.5000" in out


def test_csv_pass(tmp_path, capsys):
    p = write(tmp_path, CROSS.replace("0.5,5", "9,5"))
    assert main(["verify-external", p, "--d-min", "2"]) == 0
    assert "PASS" in capsys.readouterr().out


@pytest.mark.parametrize("text", [
    "", "a,b\n", "drone,t,x,y,z\n0,0,0,0,0\n0,1,0,0,0\n",
    "drone,t,x,y,z\n0,0,0,0,0\n0,0,1,0,0\n1,0,0,0,0\n1,1,0,0,0\n",
    "drone,t,x,y,z\n0,0,0,0,0\n0,1,nan,0,0\n1,0,0,0,0\n1,1,0,0,0\n",
    "drone,t,x,y,z\n0,0,0,0,0\n0,1,0,0,0\n1,0,0,0,0\n1,2,0,0,0\n",
    "drone,t,x,y,z\n0,0,0,0,0\n0,1,x,0,0\n1,0,0,0,0\n1,1,0,0,0\n",
])
def test_bad_csv_is_a_clean_error(tmp_path, text, capsys):
    assert main(["verify-external", write(tmp_path, text), "--d-min", "2"]) == 1
    assert "error" in capsys.readouterr().err


def test_stress_cli(tmp_path, capsys):
    exp = str(tmp_path / "e.json")
    assert main(["plan", "shows/sirb_text_100.json", "--out", exp]) == 0
    capsys.readouterr()
    rep = str(tmp_path / "r.json")
    assert main(["stress", exp, "--gnss", "shows/gnss_assumed_default.json", "--trials", "4",
                 "--report", rep]) == 0
    out = capsys.readouterr().out
    assert "SIMULATED" in out and "P(separation violation)" in out
    assert json.load(open(rep))["trials"] == 4
    assert main(["stress", exp, "--gnss", "shows/gnss_assumed_default.json", "--trials", "0"]) == 1
