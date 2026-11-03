# Decisions recorded during the assurance loop work

Author: Huda Mueen. Written during an unattended working session beginning 2026-11-03. Each entry states a decision that was made without asking, the rule that decided it, and the consequence. Entries are appended in the order the decisions were taken.

## Rules applied to every decision

1. Existing planners, the exact verifier, the second verifier and existing result files are not changed. New behaviour is added as new modules or as options whose default reproduces the old behaviour.
2. A prediction and its success criterion are committed to `docs/PREDICTIONS.md` before the experiment that tests it.
3. A number that has no source is not invented. It is either labelled as an assumption, swept as a sensitivity band, or left out.
4. Simulation and planning only. Nothing here is a safety, certification or approval claim.

## Decisions

| # | Decision | Reason |
|---|---|---|
| D1 | The local clone of `hudasol/sirb` already existed and was current with `origin/main`; it was used instead of a fresh clone. | The clone was verified current with `git fetch` before work began. The baseline test run (901 passed, 50 skipped, ruff clean) was recorded before any change. |
| D2 | Headline numbers are frozen in `tests/test_regression_headlines.py` from the committed files (viewer index, `results/differential.json`, `results/mutation.json` phase 6 block) and from a live replan of two demo shows. | These are the numbers the repository already states as results. Freezing them detects accidental change. |
| D3 | The relative GNSS error between nearby receivers: the retrieved free sources did not give a usable number (see `docs/RELATED_WORK.md`, addendum). Work package 5 therefore takes the sensitivity band route. | A long baseline (1500 km) rover study and a qualitative abstract do not bound the relative error of two neighbouring show drones. An invented number is not acceptable. |
