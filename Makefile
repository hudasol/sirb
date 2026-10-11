# One-command reproduction. Fixed seeds; trial counts match the reported numbers.
.PHONY: install test demos assignment planners viewer-data stress reproduce serve
install:
	pip install -c constraints.txt -e ".[dev]"
test:
	ruff check . && pytest -q
demos:
	python scripts/make_demos.py
planners:
	python scripts/bench_planners.py
viewer-data:
	python scripts/build_viewer_data.py
stress:
	python scripts/run_stress.py sweeps --trials 200
	python scripts/run_stress.py spacing --trials 300
	python scripts/run_stress.py contingency --trials 200
	python scripts/run_stress.py scaling
reproduce: install test demos planners viewer-data stress
serve:
	python -m http.server -d viewer 8000
