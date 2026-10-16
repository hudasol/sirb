"""Derive a GNSS parameter file from public FAA SPS accuracy statistics (stream B, phase 6).

Source: FAA GPS Standard Positioning Service Performance Analysis Report 131 (July to September
2025), Table 5-2, per site 95% horizontal and vertical position error of single frequency GPS at
WAAS reference sites. US government work (17 U.S.C. 105). The table is kept in
``data/faa_sps_pan131_table5_2.csv``; ``--extract PDF`` re-extracts it from the report with pypdf
(optional, not a Sirb dependency) and checks it against the committed copy.

What the source supports and what it does not (docs/GNSS_SOURCED.md):
* It gives ABSOLUTE position error of one standalone receiver against a surveyed point. Sirb's
  bias term is applied INDEPENDENTLY per drone, so the separation it erodes is the RELATIVE error
  of two drones. Nearby receivers tracking the same satellites share most of their error (orbit,
  clock, ionosphere, troposphere), so the relative error is expected to be smaller. No retrieved
  source gives the relative error, so using the absolute figure is a judgement and every
  parameter stays ``assumed: true``.
* The mapping from a 95% horizontal radius to a per axis standard deviation assumes a circular
  Gaussian (the model's own form): sigma = R95 / sqrt(-2 ln 0.05). The 1D vertical value maps as
  sigma_z = V95 / 1.95996. The model has one sigma for all three axes, so the horizontal value is
  used and the vertical one is reported.

    python scripts/derive_gnss_faa.py                   # writes shows/gnss_faa_sps_standalone.json
    python scripts/derive_gnss_faa.py --extract report.pdf   # also re-extract and compare
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sirb.disturbance import StressConfig  # noqa: E402
from sirb.gnss_params import from_stress_config, load, save  # noqa: E402

CSV = Path("data/faa_sps_pan131_table5_2.csv")
OUT = Path("shows/gnss_faa_sps_standalone.json")
SUMMARY = Path("results/gnss_faa_derivation.json")
URL = "https://www.nstb.tc.faa.gov/reports/FAA_SPS_PAN_Report_131_v1.0.pdf"
K_H95 = math.sqrt(-2.0 * math.log(0.05))   # 2.4477: R95 of a circular Gaussian / per axis sigma
K_V95 = 1.959963984540054                   # two sided 95% quantile of a 1D Gaussian
# The stressed scenario of docs/SENSITIVITY.md; these stay ASSUMED (no source).
SCENARIO_FRACTION = 0.25


def read_table(path: Path = CSV) -> list[dict]:
    lines = [ln for ln in path.read_text(encoding="utf-8").splitlines() if not ln.startswith("#")]
    return [{"site": r["site"], **{k: float(r[k]) for k in r if k != "site"}}
            for r in csv.DictReader(lines)]


def extract_pdf(pdf: Path) -> list[dict]:
    import pypdf  # optional; only for re-extraction
    text = "\n".join(p.extract_text() or "" for p in pypdf.PdfReader(str(pdf)).pages)
    a = text.index("Table 5-2 Horizontal and Vertical Accuracy Statistics for the Quarter")
    a = text.index("Site", a)
    b = text.index("Figure 5-1 and Figure 5-2", a)
    rows = []
    for ln in text[a:b].splitlines():
        m = re.match(r"^([A-Za-z][A-Za-z ,.]*?)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s*$", ln)
        if m:
            s, v95, h95, v99, h99 = m.groups()
            rows.append({"site": s, "v95_m": float(v95), "h95_m": float(h95),
                         "v9999_m": float(v99), "h9999_m": float(h99)})
    return rows


def derive(rows: list[dict]) -> dict:
    h = [r["h95_m"] for r in rows]
    v = [r["v95_m"] for r in rows]
    stat = {"n_sites": len(rows), "h95_median_m": statistics.median(h), "h95_min_m": min(h),
            "h95_max_m": max(h), "v95_median_m": statistics.median(v), "k_h95": K_H95,
            "k_v95": K_V95}
    stat["bias_sigma_median_m"] = stat["h95_median_m"] / K_H95
    stat["bias_sigma_min_site_m"] = stat["h95_min_m"] / K_H95
    stat["bias_sigma_max_site_m"] = stat["h95_max_m"] / K_H95
    stat["vertical_sigma_median_m"] = stat["v95_median_m"] / K_V95
    return stat


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--extract", type=Path, help="re-extract Table 5-2 from the report PDF")
    a = ap.parse_args()
    rows = read_table()
    if a.extract:
        if extract_pdf(a.extract) != rows:
            print("PDF extraction differs from", CSV)
            return 1
        print("PDF extraction matches", CSV)
    st = derive(rows)
    cfg = StressConfig(gnss_deg_fraction=SCENARIO_FRACTION,
                       gnss_bias_sigma=round(st["bias_sigma_median_m"], 4))
    prov = {
        "source_name": "FAA GPS SPS Performance Analysis Report 131 (Q3 2025), Table 5-2",
        "url": URL,
        "licence": "US government work, not subject to US copyright (17 U.S.C. 105)",
        "retrieval_date": "2026-10-16",
        "derived_by": "scripts/derive_gnss_faa.py from data/faa_sps_pan131_table5_2.csv",
        "notes": (f"bias_sigma = median over {st['n_sites']} sites of the 95% horizontal error "
                  f"({st['h95_median_m']:.3f} m) / {K_H95:.4f} (circular Gaussian), rounded to 0.1 mm; "
                  f"site range {st['bias_sigma_min_site_m']:.3f} to {st['bias_sigma_max_site_m']:.3f} m. "
                  "It is the ABSOLUTE error of one standalone receiver; the model applies it "
                  "independently per drone, so it stands in for a RELATIVE error that no retrieved "
                  "source gives. All values therefore stay assumed. degraded_fraction 0.25 is the "
                  "scenario value of docs/SENSITIVITY.md; the other values are the StressConfig "
                  "defaults (docs/METHODS.md section 5)."),
    }
    params = from_stress_config(cfg, prov, assumed=True)
    save(params, OUT)   # save validates and refuses an invalid file
    load(OUT)           # and the loader accepts what was written
    SUMMARY.write_text(json.dumps({"source": prov, "statistics": st, "rows": rows}, indent=1))
    print(json.dumps(st, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
