# Fixed seeds throughout; trial counts match the reported numbers.
PYTHON ?= python

.PHONY: help install lint test check demos planners viewer-data stress docs reproduce serve clean
.DEFAULT_GOAL := help

help:  ## list targets
	@grep -E '^[a-z-]+:.*##' $(MAKEFILE_LIST) | awk -F':.*## ' '{printf "  %-12s %s\n", $$1, $$2}'

install:  ## install with the pinned versions used for the recorded results
	pip install -c constraints.txt -e ".[dev]"

lint:  ## ruff
	ruff check .

test: lint  ## ruff and pytest
	$(PYTHON) -m pytest -q

check:  ## numbers in the reviewer documents and README must exist in results/
	$(PYTHON) scripts/check_doc_numbers.py
	$(PYTHON) scripts/check_viewer_data.py

demos:  ## regenerate the demo show files
	$(PYTHON) scripts/make_demos.py

planners:  ## planner benchmark
	$(PYTHON) scripts/bench_planners.py

viewer-data:  ## plan the demo shows and write viewer/data
	$(PYTHON) scripts/build_viewer_data.py

stress:  ## Monte Carlo experiments
	$(PYTHON) scripts/run_stress.py sweeps --trials 200
	$(PYTHON) scripts/run_stress.py spacing --trials 300
	$(PYTHON) scripts/run_stress.py contingency --trials 200
	$(PYTHON) scripts/run_stress.py scaling

docs:  ## regenerate documents from results/*.json
	$(PYTHON) scripts/make_results_doc.py
	$(PYTHON) scripts/make_sensitivity_doc.py
	$(PYTHON) scripts/make_margin_doc.py
	$(PYTHON) scripts/make_perf_doc.py
	$(PYTHON) scripts/make_ground_doc.py

reproduce: install test demos planners viewer-data stress  ## core experiments end to end

serve:  ## viewer at http://localhost:8000/
	$(PYTHON) -m http.server -d viewer 8000

clean:  ## remove caches (keeps results/.cache)
	rm -rf .pytest_cache .ruff_cache mutants
	find . -name __pycache__ -type d -prune -exec rm -rf {} +
