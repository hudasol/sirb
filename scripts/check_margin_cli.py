"""Cross check: `python -m sirb margin` reproduces a row of results/margin.json exactly.

The margin study (scripts/margin_study.py) calls ``sirb.margin.synthesise`` directly so it can
cache each evaluated scale. This script writes the study's show for one case to a file, runs the
command line tool on it in a fresh process with the same GNSS file, wind, seed, trials, target and
tolerance, and compares the tool's report with the study row. Writes results/margin_cli_check.json.

    python scripts/check_margin_cli.py        # the case that uses shows/gnss_faa_sps_standalone.json
"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from margin_study import BASE, SEED, TARGET, TOL, TRIALS  # noqa: E402
from run_stress import family_show  # noqa: E402

KEYS = ("status", "target", "factor", "required_nominal_min_separation_m", "required_margin_m",
        "d_min", "evaluations", "model_dependence", "layer")


def main() -> int:
    study = json.loads(Path("results/margin.json").read_text())
    row = next(r for r in study["rows"] if "gnss_file" in r["case"])
    case = row["case"]
    with tempfile.TemporaryDirectory() as d:
        show = Path(d) / "show.json"
        show.write_text(json.dumps(family_show(4.0, n=case["n"], big=True)))
        rep = Path(d) / "report.json"
        cmd = [sys.executable, "-m", "sirb", "margin", str(show), "--gnss", case["gnss_file"],
               "--w20", str(BASE["w20"]), "--target", str(TARGET), "--trials", str(TRIALS),
               "--seed", str(SEED), "--tol", str(TOL), "--report", str(rep)]
        p = subprocess.run(cmd, capture_output=True, text=True)
        cli = json.loads(rep.read_text()) if rep.exists() else None
    diffs = [k for k in KEYS if cli is None or cli.get(k) != row.get(k)]
    out = {"case": case, "command": "python -m sirb margin <show> " + " ".join(cmd[5:-2]),
           "returncode": p.returncode, "stdout": p.stdout.strip().splitlines(),
           "compared_keys": list(KEYS), "differing_keys": diffs, "identical": not diffs}
    Path("results/margin_cli_check.json").write_text(json.dumps(out, indent=1))
    print("identical" if not diffs else f"differs in {diffs}")
    return 0 if not diffs else 1


if __name__ == "__main__":
    sys.exit(main())
