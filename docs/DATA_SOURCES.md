# Data sources and licenses

Updated as dependencies are added. Python package licenses below were read from installed package metadata on 2026-10-11 (numpy 2.5.3, scipy 1.18.1, pytest 9.1.1, hypothesis 6.168.5). three.js is still unverified until vendored.

## Code dependencies

| Package | Use | License | Verified how |
|---|---|---|---|
| numpy | arrays, polynomial roots | BSD-3-Clause AND 0BSD AND MIT AND Zlib AND CC0-1.0 (metadata expression) | package metadata |
| scipy | assignment, spatial queries | BSD-3-Clause (license text read; wheel also bundles other components, e.g. OpenBLAS BSD-3-Clause) | package metadata |
| pytest | tests | MIT | package metadata |
| hypothesis | property-based tests (dev only, not redistributed) | MPL-2.0 | package metadata |
| ruff | lint (dev only) | MIT | package metadata |
| three.js | viewer | MIT (expected) | to verify when vendored |

three.js "expected" means stated from general knowledge; replace with the verified value when vendored, before v0.9.0.

## Data

No external datasets. All shapes are procedural or parametric, generated in-repo from seeds. No logos, trademarks or copyrighted designs. Text shapes use a stroke font defined in the repo.

## Reference material (not redistributed)

Papers and web pages cited in `docs/RESEARCH.md` are linked, not copied. No Skybrush code or file format is used; Skybrush components are GPL-3.0 (per GitHub pages read), which is incompatible with silently adapting them into an MIT project.

## Parameters

Vehicle and GNSS parameters carry their sources or the label ASSUMPTION in `docs/RESEARCH.md` sections 3 and 5.
