# Licence proposal (for the owner to decide; nothing here is legal advice)

Current state: `LICENSE` is MIT for the whole repository.

## Proposal
| Part | Proposed licence | Reason |
|---|---|---|
| Code (`sirb/`, `scripts/`, `tests/`, `viewer/` except `viewer/vendor/`) | **Apache-2.0** (or keep MIT) | Same permissive spirit as MIT, plus an explicit patent grant and a NOTICE mechanism, which matters for a tool others might build on in robotics. Apache-2.0 is also the licence of `hudasol/marsad`, so the two repositories would match. MIT is simpler and is fully acceptable if patent language is not a concern. |
| Documentation, figures and screenshots (`docs/`, `README.md`, `viewer/screenshots/`) | **CC BY 4.0** | Lets others reuse and cite the analysis with attribution. |
| Show files and result JSON (`shows/`, `results/`) | **CC BY 4.0** (procedurally generated, original shapes) | Data is better covered by a data licence than a software licence. |
| Vendored third party files (`viewer/vendor/`, `scripts/vendor/`) | Their own licences, kept in place | Recorded in `docs/DATA_SOURCES.md`. |

## What changing would involve
Replacing `LICENSE` text, adding `NOTICE`, adding a short licence table to the README, and adding SPDX
headers if wanted. Because the author is the sole copyright holder of the repository's own files, the
change is her decision to make; third party files keep their licences. Not a licence for any real
flight use: the safety statement applies whatever the licence.
