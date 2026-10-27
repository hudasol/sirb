"""Derive a literature anchored GNSS parameter file for the error that remains after common mode
cancellation (stream D, phase 6).

Source: Carvalho et al. (2023), Sensors 23(21):8835, CC BY 4.0; Table 1 is kept in
``data/carvalho2023_sensors_table1.csv``. In one experiment a low cost receiver (u-blox F9R) on a
moving rover was positioned (a) standalone and (b) by single differenced pseudoranges against a
reference station 1.184 km away, which cancels the errors common to both sites (orbit, satellite
clock, most of the atmosphere). The mean horizontal error fell from 1.202 m to 0.713 m.

Why this is closer to what Sirb needs than the FAA figure (docs/GNSS_SOURCED.md): Sirb applies the
bias independently to each degraded drone, so it should be the part of one drone's error that is
not shared with its neighbours. Case (b) measures roughly that part for one low cost receiver.

Why every value still stays ``assumed: true`` (rule of docs/GNSS_PARAMS.md):
* the mapping from a mean horizontal error to a per axis sigma assumes a circular Gaussian
  (Rayleigh distributed radius): sigma = mean / sqrt(pi / 2);
* the reference was a high grade station, not a second low cost receiver; two low cost drones
  would each contribute their own non common error;
* the rover moved under tree canopy (strong multipath) and was filtered by an extended Kalman
  filter; one run of 800 s; GPS L1 only. None of this is the target hardware or setting.
It is a literature value, not a measurement of any show drone.

    python scripts/derive_gnss_relative.py     # writes shows/gnss_relative_literature.json
"""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sirb.disturbance import StressConfig  # noqa: E402
from sirb.gnss_params import from_stress_config, load, save  # noqa: E402

CSV = Path("data/carvalho2023_sensors_table1.csv")
OUT = Path("shows/gnss_relative_literature.json")
SUMMARY = Path("results/gnss_relative_derivation.json")
K_RAYLEIGH_MEAN = math.sqrt(math.pi / 2.0)   # mean radius of a circular Gaussian / per axis sigma
SCENARIO_FRACTION = 0.25                     # scenario value of docs/SENSITIVITY.md (assumed)


def read_table(path: Path = CSV) -> dict:
    lines = [ln for ln in path.read_text(encoding="utf-8").splitlines() if not ln.startswith("#")]
    return {(r["solution"], r["component"]): (float(r["mean_m"]), float(r["std_m"]))
            for r in csv.DictReader(lines)}


def main() -> int:
    t = read_table()
    rel_h, sa_h = t[("RGNSS", "Horizontal")][0], t[("SGNSS", "Horizontal")][0]
    sigma = rel_h / K_RAYLEIGH_MEAN
    st = {"relative_horizontal_mean_m": rel_h, "standalone_horizontal_mean_m": sa_h,
          "relative_over_standalone": rel_h / sa_h, "k_rayleigh_mean": K_RAYLEIGH_MEAN,
          "bias_sigma_m": sigma, "baseline_km": 1.184, "epochs": 800}
    cfg = StressConfig(gnss_deg_fraction=SCENARIO_FRACTION, gnss_bias_sigma=round(sigma, 4))
    prov = {
        "source_name": "Carvalho et al. (2023), Sensors 23(21):8835, Table 1",
        "url": "https://doi.org/10.3390/s23218835",
        "licence": "CC BY 4.0 (stated on the article page)",
        "retrieval_date": "2026-10-18",
        "derived_by": "scripts/derive_gnss_relative.py from data/carvalho2023_sensors_table1.csv",
        "notes": (f"bias_sigma = mean horizontal error after common mode cancellation over a "
                  f"{st['baseline_km']} km baseline ({rel_h} m, against {sa_h} m standalone in the "
                  f"same run) / sqrt(pi/2) (circular Gaussian), rounded to 0.1 mm. Literature value "
                  "for one low cost receiver on a moving rover with a high grade reference, not a "
                  "measurement of show drones; mapping and setting are judgements, so all values stay "
                  "assumed. Other values: degraded_fraction 0.25 (scenario of docs/SENSITIVITY.md) "
                  "and the StressConfig defaults."),
    }
    save(from_stress_config(cfg, prov, assumed=True), OUT)
    load(OUT)
    SUMMARY.write_text(json.dumps({"source": prov, "statistics": st}, indent=1) + "\n",
                       encoding="utf-8")
    print(json.dumps(st, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
