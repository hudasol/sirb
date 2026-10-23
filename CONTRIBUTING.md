# Contributing

Sirb is a simulation and planning tool. Changes that add flight control, radio or MAVLink code, or payload features, will not be accepted.

## Setup

```
pip install -c constraints.txt -e ".[dev]"
make test
```

`constraints.txt` pins the versions used for the recorded results. Use Python 3.12 or newer.

## Rules for changes

- Keep the verifier (`sirb/verify.py`) free of imports from the planners. `sirb/verify2.py` must not import `verify.py`.
- A change that moves a number under `results/` must regenerate that file with its script and the documents derived from it (`scripts/make_*_doc.py`). Documents under `docs/` that are generated say so in their first lines; edit the script, not the output.
- New experiments need a fixed seed and a per unit cache under `results/.cache` so an interrupted run resumes.
- Add a test for each fix. Verifier changes also need the differential suite (`tests/test_verify2_differential.py`) to stay clean.
- Assumed parameters stay labelled as assumptions until a source is cited in `docs/DATA_SOURCES.md`.
- `ruff check .` and `pytest -q` must pass. CI also rebuilds the viewer data and compares it with the committed copy.

## Commits

One logical change per commit, with a message saying what changed and why.
