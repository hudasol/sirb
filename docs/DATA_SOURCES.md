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
| three.js 0.170.0 (`three.module.min.js`, `addons/OrbitControls.js`) | viewer 3D rendering, vendored in `viewer/vendor/` (no CDN) | MIT ("Copyright 2010-2024 three.js authors") | `license` field of the npm package and its LICENSE file, copied to `viewer/vendor/LICENSE` (2026-10-11) |

Transitive dev-only dependencies (installed by pytest and hypothesis; not imported by `sirb`; licenses from package metadata, review round 6): iniconfig 2.3.1 MIT, packaging 26.3 Apache-2.0 OR BSD-2-Clause, pluggy 1.6.0 MIT, pygments 2.21.0 BSD-2-Clause, sortedcontainers 2.4.0 Apache-2.0. Runtime dependencies are numpy and scipy only (`pip check`: no broken requirements). Exact versions used are in `constraints.txt`. `pytest-cov` 7.1.0 was installed ad hoc for the coverage run in review round 6; it is not a declared dependency.

Compiled wheels of numpy and scipy bundle `libgfortran` (GPL-3.0 with the GCC Runtime Library Exception, per the bundled LICENSE text), `libquadmath` (LGPL) and OpenBLAS (BSD-3-Clause). Sirb does not redistribute these wheels; users install them from PyPI.

Vendored viewer files are byte-identical to the npm package `three@0.170.0` (tarball integrity checked against the registry's sha512 on 2026-10-11). SHA-256 of the vendored copies (a test fails if they change):

| File | SHA-256 |
|---|---|
| `viewer/vendor/three.module.min.js` | `08fd7545d13d2c7fb65ab691530a802dafefd638596501854f267d0fb13c39e7` |
| `viewer/vendor/addons/OrbitControls.js` | `80efaadea4f8a636a65fb0bd08bfef62f3d93a0bb94e2e7500f23176c5c07f4e` |
| `viewer/vendor/LICENSE` | `4c40a1ef62450b857c3b2aaf294936304cd552d965fbcd9d32d4c5bcf4ba4454` |

The viewer has no other JavaScript dependencies (vanilla ES modules and an importmap).

## Data

No external datasets. All shapes are procedural or parametric, generated in-repo from seeds. No logos, trademarks or copyrighted designs. Text shapes use a stroke font defined in the repo.

## Reference material (not redistributed)

Papers and web pages cited in `docs/RESEARCH.md` are linked, not copied. No Skybrush code or file format is used; Skybrush components are GPL-3.0 (per GitHub pages read), which is incompatible with silently adapting them into an MIT project.

## Parameters

Vehicle and GNSS parameters carry their sources or the label ASSUMPTION in `docs/RESEARCH.md` sections 3 and 5.
