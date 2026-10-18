# Data sources and licenses

Updated as dependencies are added. Python package licenses below were read from installed package metadata on 2026-10-11 (numpy 2.5.3, scipy 1.18.1, pytest 9.1.1, hypothesis 6.168.5). three.js was verified when vendored (below).

## Code dependencies

| Package | Use | License | Verified how |
|---|---|---|---|
| numpy | arrays, polynomial roots | BSD-3-Clause AND 0BSD AND MIT AND Zlib AND CC0-1.0 (metadata expression) | package metadata |
| scipy | assignment, spatial queries | BSD-3-Clause (license text read; wheel also bundles other components, e.g. OpenBLAS BSD-3-Clause) | package metadata |
| pytest | tests | MIT | package metadata |
| hypothesis | property-based tests (dev only, not redistributed) | MPL-2.0 | package metadata |
| ruff | lint (dev only) | MIT | package metadata |
| mutmut 3.8.0 | mutation testing of the verifier and planners (dev only, not imported by `sirb`, not redistributed; `docs/MUTATION.md`) | BSD-3-Clause (the `License-Expression` field of the installed package metadata, read 2026-10-12) | package metadata |
| three.js 0.170.0 (`three.module.min.js`, `addons/OrbitControls.js`) | viewer 3D rendering, vendored in `viewer/vendor/` (no CDN) | MIT ("Copyright 2010-2024 three.js authors") | `license` field of the npm package and its LICENSE file, copied to `viewer/vendor/LICENSE` (2026-10-11) |
| axe-core 4.14.0 (`axe.min.js`, `LICENSE`, `package.json`) | accessibility test runs only (`scripts/axe_check.py` injects it with Playwright); kept in `scripts/vendor/axe-core/`, not part of the shipped site | MPL-2.0 (`license` field of the npm package metadata and its LICENSE file, copied alongside) | `npm pack axe-core` tarball, 2026-10-12; SHA-256 of `axe.min.js` `20c09fe157a8a34a30e241aaa1fcdade657734f08ab379ecfbeb7d45cc46e878` |
| jsonschema 4.26.0 with attrs 26.1.0, jsonschema-specifications 2025.9.1, referencing 0.37.0, rpds-py 2026.9.1 | dev only: `tests/test_gnss_params.py` checks the shipped GNSS files against `sirb/schemas/gnss_params.schema.json`; not imported by `sirb`, not redistributed | MIT for all five (`License-Expression` or `License` of the installed package metadata, read 2026-10-18) | package metadata |
| pypdf 6.20.1 | optional re-extraction of the FAA accuracy table from the report PDF (`scripts/derive_gnss_faa.py --extract`); dev only, not a declared dependency, not redistributed | BSD-3-Clause (`License-Expression` of the installed package metadata, read 2026-10-16) | package metadata |
| Playwright (Python) 1.56.0 with its preinstalled Chromium; Pillow 12.3.0 | viewer test, screenshot and benchmark scripts only (dev only, not redistributed, not declared dependencies) | Playwright Apache-2.0 (`License-Expression`); Pillow MIT-CMU (`License-Expression`) | installed package metadata, 2026-10-12 |

Transitive dev-only dependencies (installed by pytest and hypothesis; not imported by `sirb`; licenses from package metadata, review round 6): iniconfig 2.3.1 MIT, packaging 26.3 Apache-2.0 OR BSD-2-Clause, pluggy 1.6.0 MIT, pygments 2.21.0 BSD-2-Clause, sortedcontainers 2.4.0 Apache-2.0. Runtime dependencies are numpy and scipy only (`pip check`: no broken requirements). Exact versions used are in `constraints.txt`. mutmut 3.8.0 pulls in click 8.5.0 BSD-3-Clause, coverage 7.16.2 Apache-2.0, libcst 1.9.0 MIT (its metadata adds that a few files derived from the Python standard library are PSF-licensed), setproctitle 1.3.8 BSD-3-Clause and textual 8.2.8 MIT (all from installed package metadata, 2026-10-12); all are dev-only. `pytest-cov` 7.1.0 was installed ad hoc for the coverage run in review round 6; it is not a declared dependency.

Compiled wheels of numpy and scipy bundle `libgfortran` (GPL-3.0 with the GCC Runtime Library Exception, per the bundled LICENSE text), `libquadmath` (LGPL) and OpenBLAS (BSD-3-Clause). Sirb does not redistribute these wheels; users install them from PyPI.

Vendored viewer files are byte-identical to the npm package `three@0.170.0` (tarball integrity checked against the registry's sha512 on 2026-10-11). SHA-256 of the vendored copies (a test fails if they change):

| File | SHA-256 |
|---|---|
| `viewer/vendor/three.module.min.js` | `08fd7545d13d2c7fb65ab691530a802dafefd638596501854f267d0fb13c39e7` |
| `viewer/vendor/addons/OrbitControls.js` | `80efaadea4f8a636a65fb0bd08bfef62f3d93a0bb94e2e7500f23176c5c07f4e` |
| `viewer/vendor/LICENSE` | `4c40a1ef62450b857c3b2aaf294936304cd552d965fbcd9d32d4c5bcf4ba4454` |

Services (free tier, not redistributed): GitHub hosts the repository, GitHub Actions runs CI and GitHub Pages would serve the viewer; their terms are GitHub's, and no paid feature is used. The Python standard library is under the PSF License.

The viewer has no other JavaScript dependencies (vanilla ES modules and an importmap).

## Data

All shapes are procedural or parametric, generated in-repo from seeds. No logos, trademarks or copyrighted designs. Text shapes use a stroke font defined in the repo.

One external data table is used (phase 6, `docs/GNSS_SOURCED.md`):

| Data | Use | Licence | Retrieved |
|---|---|---|---|
| FAA GPS SPS Performance Analysis Report 131 (July to September 2025), Table 5-2, per site 95% horizontal and vertical position error; <https://www.nstb.tc.faa.gov/reports/FAA_SPS_PAN_Report_131_v1.0.pdf> | magnitude of the GNSS bias in `shows/gnss_faa_sps_standalone.json`; the 26 rows are copied to `data/faa_sps_pan131_table5_2.csv` | Produced by the FAA William J. Hughes Technical Center, a US federal agency. The report states no licence; works of the US federal government are not subject to US copyright (17 U.S.C. 105). Outside the US the status may differ; the table is reproduced with attribution. | 2026-10-16, PDF SHA-256 `fd3e66833160c8a4920e9bb9f835a58a6d23cb50baeeeb335e6e247b5ccc3055` |
| Carvalho, Silva, Pacheco and Campos (2023), Performance Analysis of Relative GPS Positioning for Low-Cost Receiver-Equipped Agricultural Rovers, Sensors 23(21):8835, Table 1; <https://doi.org/10.3390/s23218835> (read on PubMed Central, PMC10650014) | literature anchor for the error left after common mode cancellation, `shows/gnss_relative_literature.json`; the ten table values are copied to `data/carvalho2023_sensors_table1.csv` | CC BY 4.0, as stated on the article page; reproduced with attribution | 2026-10-18; the article page was read on PubMed Central, the PDF could not be downloaded for hashing (HTTP 403 from both MDPI and Europe PMC), so no hash is recorded |

No other external dataset is used. `hudasol/marsad` and the Tashwish project were not used (`docs/GNSS_PARAMS.md`).

## Reference material (not redistributed)

Papers and web pages cited in `docs/RESEARCH.md` are linked, not copied. No Skybrush code or file format is used; Skybrush components are GPL-3.0 (per GitHub pages read), which is incompatible with silently adapting them into an MIT project.

## Parameters

Vehicle and GNSS parameters carry their sources or the label ASSUMPTION in `docs/RESEARCH.md` sections 3 and 5.
