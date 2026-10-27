#!/usr/bin/env python
"""Manual mutation check of recorded survivors (mutmut does not run on Windows).

``results/mutation.json`` lists, per source file, the mutants that survived the test selection,
each with a textual change ``function: old line  ->  new line`` (the physical source lines that
differ between the original function and the mutant; several lines are joined by `` / ``). For
every survivor this tool

1. finds the old lines inside the named function of the *current* source (consecutive stripped
   physical lines; if the text is absent the survivor is ``not_applicable`` because the code was
   removed or rewritten; if it occurs several times the occurrences are assigned to the survivors
   with the identical change in mutant-number order, which is the order mutmut generates them in,
   and only when the counts agree, otherwise it is ``not_applicable`` as ambiguous),
2. builds the mutated source of the module (the changed physical lines replaced),
3. runs the given pytest selection against it, either (``--mode process``) in a new interpreter
   whose ``sirb`` is a throw-away copy of the package with the one file replaced
   (``PYTHONPATH`` points at the copy), or (``--mode worker``, the default) in a persistent worker
   that executes the mutated source in a scratch namespace and copies the code objects and
   defaults of its functions into the real module, restoring them afterwards (start-up and
   imports cost about a minute per interpreter on a loaded machine); a timeout applies to both,
4. records ``killed`` (a test failed), ``survived`` (all passed), ``timeout`` or ``error``.

The selection is first run on the unmutated code; a worker stops if it does not pass. Results go
to a JSON file that is read back (and extended) on the next call, so the stages of a triage
(old selection, new tests, slow tests) accumulate in each entry's ``history``. Only entries whose
current status is ``survived`` (or ``pending`` or ``timeout``) are run again. ``--probe NAME``
records a result without changing the status (for example to ask which kills a subset of the
tests keeps). ``--finalize OUT REASONS`` adds each entry's final class from a reasons file.

Example::

    python scripts/check_survivors.py --files sirb/verify2.py --label old \\
        --tests "tests/test_verify2_differential.py::test_hold_semantics_match"
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SEP = "  ->  "
JOIN = " / "
_PRINT = threading.Lock()


# ------------------------------------------------------------------------------ locating

def _function_span(tree: ast.AST, name: str):
    spans = [(n.lineno, n.end_lineno) for n in ast.walk(tree)
             if isinstance(n, ast.FunctionDef) and n.name == name]
    return spans[0] if len(spans) == 1 else None


def _find_runs(lines: list[str], span, old: str):
    """Start indices (0-based, absolute) of consecutive stripped physical lines in ``span`` whose
    ``" / "``-join equals ``old``; the number of physical lines is tried from 1 up to 6."""
    lo, hi = span[0] - 1, span[1]
    hits = []
    for n in range(1, 7):
        for s in range(lo, hi - n + 1):
            if JOIN.join(x.strip() for x in lines[s:s + n]) == old:
                hits.append((s, n))
    return hits


def _def_default_mutation(lines: list[str], span):
    """mutmut_1 of a function whose only change sits on the ``def`` line (which the recorded
    change text filters out): assumed to be the first default value increased by one. Returns the
    mutated lines or None."""
    i = span[0] - 1
    j = i
    while not lines[j].rstrip().endswith(":") and j < span[1] - 1:     # end of the signature
        j += 1
    sig = "".join(lines[i:j + 1])
    m = re.search(r"=\s*(-?\d+(?:\.\d+)?)", sig)
    if not m:
        return None
    val = m.group(1)
    new = str(int(val) + 1) if re.fullmatch(r"-?\d+", val) else repr(float(val) + 1.0)
    out = list(lines)
    for k in range(i, j + 1):
        if re.search(r"=\s*" + re.escape(val), out[k]):
            out[k] = re.sub(r"=(\s*)" + re.escape(val), lambda mm: "=" + mm.group(1) + new, out[k],
                            count=1)
            return out
    return None


def _apply(lines: list[str], text: str, start: int, n: int, new: str):
    """Replace ``n`` physical lines from ``start`` by ``new`` (split on `` / `` into physical lines
    when that is needed for the result to parse); None if nothing parses or nothing changes."""
    indent = re.match(r"\s*", lines[start]).group(0)
    nl = "\r\n" if lines[start].endswith("\r\n") else "\n"
    cands = [[new]]
    for p in range(min(len(new.split(JOIN)), 6), 1, -1):
        cands.append(new.split(JOIN, p - 1))
    for cand in cands:
        body = "".join(indent + c + nl for c in cand)
        mutated = "".join(lines[:start]) + body + "".join(lines[start + n:])
        try:
            ast.parse(mutated)
        except SyntaxError:
            continue
        if mutated != text:
            return mutated
    return None


def mutate(path: Path, surv: list[dict]):
    """Return {mutant: (mutated_text or None, locate_note)} for the survivors of one file."""
    text = path.read_text()
    lines = text.splitlines(keepends=True)
    tree = ast.parse(text)
    out: dict[str, tuple] = {}
    groups: dict[tuple, list[dict]] = {}
    for s in surv:
        func, _, rest = s["change"].partition(": ")
        old, _, new = rest.partition(SEP)
        groups.setdefault((func, old, new), []).append(s)
    placed: dict[str, dict[int, int]] = {}          # function -> {mutant number: start line}
    deferred = []                                    # (survivor, func, hits, new)
    num = lambda s: int(s["mutant"].rsplit("_", 1)[1])   # noqa: E731
    for (func, old, new), members in groups.items():
        members.sort(key=num)
        span = _function_span(tree, func)
        if span is None:
            for s in members:
                out[s["mutant"]] = (None, f"function {func} not found (code removed or rewritten)")
            continue
        if old == "" and new == "(removed)":
            for s in members:
                nl = _def_default_mutation(lines, span)
                out[s["mutant"]] = (("".join(nl) if nl else None),
                                    "def-line default value + 1 (assumed: the def line is filtered "
                                    "out of the recorded change)" if nl else "no default value")
            continue
        if new == "(removed)":
            for s in members:
                out[s["mutant"]] = (None, "statement removed: not supported")
            continue
        hits = sorted(set(_find_runs(lines, span, old)))
        if not hits:
            for s in members:
                out[s["mutant"]] = (None, "old line not found in the current function "
                                          "(code removed or rewritten in phase 6)")
            continue
        if len(hits) == len(members):
            for k, s in enumerate(members):
                start, n = hits[k]
                placed.setdefault(func, {})[num(s)] = start
                mut = _apply(lines, text, start, n, new)
                note = "exact" if len(hits) == 1 else f"occurrence {k + 1} of {len(hits)}"
                out[s["mutant"]] = (mut, note if mut else "mutated text does not parse")
        elif len(members) == 1:
            deferred.append((members[0], func, hits, new))
        else:
            for s in members:
                out[s["mutant"]] = (None, f"old line occurs {len(hits)} times, {len(members)} "
                                          "survivors with this change: ambiguous")
    # A lone survivor whose old line occurs several times: mutants are numbered in source order,
    # so it must sit between the nearest placed survivors with a lower and a higher number.
    for s, func, hits, new in deferred:
        pl = placed.get(func, {})
        lower = max([ln for k, ln in pl.items() if k < num(s)], default=-1)
        upper = min([ln for k, ln in pl.items() if k > num(s)], default=10 ** 9)
        # a mutation of a whole statement keeps the order only among distinct statements, so use the
        # nearest neighbours' lines as bounds
        cand = [h for h in hits if lower <= h[0] <= upper]
        if len(cand) == 1:
            mut = _apply(lines, text, cand[0][0], cand[0][1], new)
            out[s["mutant"]] = (mut, f"occurrence at line {cand[0][0] + 1}, fixed by the order of "
                                     "the neighbouring mutants" if mut else "does not parse")
        else:
            out[s["mutant"]] = (None, f"old line occurs {len(hits)} times, 1 survivor: ambiguous")
    return out


# ------------------------------------------------------------------------------ running

def _kill_tree(proc: subprocess.Popen) -> None:
    if os.name == "nt":
        subprocess.run(["taskkill", "/T", "/F", "/PID", str(proc.pid)], capture_output=True)
    else:
        proc.kill()


def run_tests(python: str, workdir: Path, tests: list[str], timeout: float):
    """``process`` mode: pytest in a new interpreter whose ``sirb`` is the temporary copy."""
    env = dict(os.environ)
    env["PYTHONPATH"] = str(workdir)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    cmd = [python, "-m", "pytest", *tests, "-q", "-x", "-rf", "-p", "no:cacheprovider",
           "--rootdir", str(ROOT)]
    t0 = time.time()
    proc = subprocess.Popen(cmd, cwd=str(workdir), env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, errors="replace")
    try:
        out, _ = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        _kill_tree(proc)
        try:
            out, _ = proc.communicate(timeout=20)
        except subprocess.TimeoutExpired:
            out = ""
        return "timeout", None, time.time() - t0, out or ""
    rc = proc.returncode
    status = {0: "survived", 1: "killed"}.get(rc, "error")
    killer = None
    if rc == 1:
        m = re.search(r"^FAILED (\S+)", out, re.M)
        killer = m.group(1) if m else None
        if killer and "tests" in killer:                       # path relative to the temp copy
            killer = "tests/" + killer.replace("\\", "/").split("tests/", 1)[-1]
    return status, killer, time.time() - t0, out


def make_copy(base: Path, rel: str, text: str | None) -> Path:
    """A throw-away copy of the ``sirb`` package (with ``rel`` replaced by ``text`` if given)."""
    work = Path(tempfile.mkdtemp(prefix="mut_", dir=str(base)))
    shutil.copytree(ROOT / "sirb", work / "sirb", ignore=shutil.ignore_patterns("__pycache__"))
    if text is not None:
        (work / rel).write_text(text)
    return work


def check_import(python: str, work: Path) -> None:
    env = dict(os.environ, PYTHONPATH=str(work))
    r = subprocess.run([python, "-c", "import sirb; print(sirb.__file__)"], cwd=str(work),
                       env=env, capture_output=True, text=True)
    got = Path(r.stdout.strip()).resolve()
    if work.resolve() not in got.parents:
        raise SystemExit(f"sirb is imported from {got}, not from the temporary copy {work}")


# ---- worker mode: the interpreter start-up and the numpy/scipy imports take about a minute on
# a loaded machine, so one worker imports everything once and then, for each mutant, executes the
# mutated source in a scratch namespace and copies the code objects (and defaults) of its
# functions into the real module's functions. Every reference to those functions, whether
# through the module or bound earlier by ``from module import f``, then runs the mutant. The
# originals are restored afterwards. A watchdog thread ends the worker on a timeout; the
# controller restarts it for the remaining mutants.

def worker_main(job_path: str, result_path: str) -> int:
    import importlib
    import threading
    import types
    from contextlib import redirect_stderr, redirect_stdout
    from io import StringIO

    job = json.loads(Path(job_path).read_text())
    sys.path.insert(0, str(ROOT))
    os.chdir(ROOT)
    import pytest

    def emit(rec: dict) -> None:
        with open(result_path, "a") as fh:
            fh.write(json.dumps(rec) + "\n")

    class Collect:
        killer = None

        def pytest_runtest_logreport(self, report):
            if report.failed and Collect.killer is None:
                Collect.killer = report.nodeid

    def run() -> tuple[str, str | None]:
        Collect.killer = None
        buf = StringIO()
        with redirect_stdout(buf), redirect_stderr(buf):
            rc = pytest.main([*job["tests"], "-q", "-x", "-p", "no:cacheprovider",
                              "--rootdir", str(ROOT)], plugins=[Collect()])
        return {0: "survived", 1: "killed"}.get(int(rc), "error"), Collect.killer

    mods: dict[str, tuple] = {}

    def module_state(rel: str):
        if rel not in mods:
            name = rel[:-3].replace("/", ".")
            mod = importlib.import_module(name)
            funcs = {k: v for k, v in vars(mod).items()
                     if isinstance(v, types.FunctionType) and v.__module__ == name}
            saved = {k: (f.__code__, f.__defaults__, f.__kwdefaults__) for k, f in funcs.items()}
            mods[rel] = (mod, funcs, saved)
        return mods[rel]

    st, _ = run()
    emit({"baseline": st})
    if st != "survived":
        return 2
    for m in job["mutants"]:
        mod, funcs, saved = module_state(m["file"])
        text = Path(m["text_path"]).read_text()
        ns = {"__name__": mod.__name__, "__file__": mod.__file__}
        t0 = time.time()
        done = threading.Event()

        def watchdog(name=m["mutant"]):
            if not done.wait(job["timeout"]):
                emit({"mutant": name, "status": "timeout", "seconds": job["timeout"]})
                os._exit(7)

        threading.Thread(target=watchdog, daemon=True).start()
        try:
            exec(compile(text, mod.__file__, "exec"), ns)
            for k, f in funcs.items():
                g = ns[k]
                f.__code__, f.__defaults__, f.__kwdefaults__ = (
                    g.__code__, g.__defaults__, g.__kwdefaults__)
            status, killer = run()
        except BaseException as exc:                       # noqa: BLE001
            status, killer = "error", repr(exc)
        finally:
            for k, f in funcs.items():
                f.__code__, f.__defaults__, f.__kwdefaults__ = saved[k]
            for mod_, _f, _s in mods.values():            # module caches filled by a mutant
                for key, val in vars(mod_).items():
                    if key.endswith("_CACHE") and isinstance(val, dict):
                        val.clear()
            done.set()
        emit({"mutant": m["mutant"], "status": status, "killed_by": killer,
              "seconds": round(time.time() - t0, 1)})
    return 0


def run_workers(a, todo, tests, state, out):
    """Distribute ``todo`` over ``a.jobs`` workers and merge their result files as they arrive."""
    base = Path(a.workdir)
    jobdir = Path(tempfile.mkdtemp(prefix="job_", dir=str(base)))
    queue = [(e, text) for e, text in todo]
    for i, (_e, text) in enumerate(queue):
        (jobdir / f"m{i}.py").write_text(text)
    groups = [list(range(w, len(queue), a.jobs)) for w in range(a.jobs)]
    groups = [g for g in groups if g]
    counts: dict[str, int] = {}

    def record(e, rec):
        r = {"stage": a.label, "status": rec["status"], "seconds": rec.get("seconds"),
             "tests": a.tests}
        if rec.get("killed_by"):
            r["killed_by"] = rec["killed_by"]
        if a.probe:                       # side experiment: do not change the triage status
            with _PRINT:
                e.setdefault("probes", {})[a.probe] = {k: v for k, v in r.items() if k != "stage"}
                counts[rec["status"]] = counts.get(rec["status"], 0) + 1
                print(f"[{a.probe}] {e['mutant'].split('.')[-1]:44s} {rec['status']:9s} "
                      f"{rec.get('seconds')}s {rec.get('killed_by') or ''}", flush=True)
                out.write_text(json.dumps(state, indent=1) + "\n")
            return
        with _PRINT:
            e["history"].append(r)
            e["status"] = rec["status"]
            e["tests"] = [rec["killed_by"]] if rec.get("killed_by") and rec["status"] == "killed" \
                else []
            counts[rec["status"]] = counts.get(rec["status"], 0) + 1
            print(f"{e['mutant'].split('.')[-1]:50s} {rec['status']:9s} "
                  f"{rec.get('seconds')}s {rec.get('killed_by') or ''}", flush=True)
            out.write_text(json.dumps(state, indent=1) + "\n")

    def drive(gid_idx):
        idxs = list(gid_idx)
        attempt = 0
        while idxs and attempt < 8:
            attempt += 1
            tag = f"{os.getpid()}_{idxs[0]}_{attempt}"
            job = {"tests": tests, "timeout": a.timeout,
                   "mutants": [{"mutant": queue[i][0]["mutant"], "file": queue[i][0]["file"],
                                "text_path": str(jobdir / f"m{i}.py")} for i in idxs]}
            jp, rp = jobdir / f"job_{tag}.json", jobdir / f"res_{tag}.jsonl"
            jp.write_text(json.dumps(job))
            env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
            proc = subprocess.Popen([a.python, str(Path(__file__).resolve()), "--worker",
                                     str(jp), str(rp)], env=env, cwd=str(ROOT),
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            seen = 0
            while True:
                time.sleep(2)
                if rp.exists():
                    lines = rp.read_text().splitlines()
                    for ln in lines[seen:]:
                        rec = json.loads(ln)
                        if "baseline" in rec:
                            if rec["baseline"] != "survived":
                                print("baseline FAILED in a worker:", rec)
                                proc.kill()
                                return
                            continue
                        i = next(j for j in idxs if queue[j][0]["mutant"] == rec["mutant"])
                        record(queue[i][0], rec)
                        idxs.remove(i)
                    seen = len(lines)
                if proc.poll() is not None:
                    break
            if rp.exists():
                for ln in rp.read_text().splitlines()[seen:]:
                    rec = json.loads(ln)
                    if "mutant" in rec:
                        i = next((j for j in idxs if queue[j][0]["mutant"] == rec["mutant"]), None)
                        if i is not None:
                            record(queue[i][0], rec)
                            idxs.remove(i)
            if idxs and proc.returncode not in (0, 7):
                i = idxs.pop(0)          # the worker died on this mutant without a record
                record(queue[i][0], {"status": "error", "killed_by": f"worker exit {proc.returncode}"})

    with ThreadPoolExecutor(max_workers=len(groups)) as pool:
        list(pool.map(drive, groups))
    shutil.rmtree(jobdir, ignore_errors=True)
    return counts


# ------------------------------------------------------------------------------ main

def load_state(out: Path) -> dict:
    if out.exists():
        return json.loads(out.read_text())
    return {"note": "Generated by scripts/check_survivors.py; see its docstring.", "entries": []}


def finalize(out_path: str, reasons_path: str) -> int:
    """Write each entry's final class: killed, not_applicable, or the class and one-sentence
    reason of the rule in the reasons file that names the mutant (otherwise ``open``)."""
    out = Path(out_path)
    state = json.loads(out.read_text())
    rules = json.loads(Path(reasons_path).read_text())["rules"]
    named = {}
    for rule in rules:
        for m in rule["mutants"]:
            named[m] = rule
    summary: dict = {}
    for e in state["entries"]:
        short = e["mutant"].split(".")[-1]
        stages = [h["stage"] for h in e["history"]]
        if short in named and named[short].get("overrides_killed"):
            fin = {"class": named[short]["class"], "reason": named[short]["reason"]}
        elif e["status"] == "killed":
            probes = e.get("probes", {})
            fin = {"class": "killed", "reason": "killed by " + (e["tests"][0] if e["tests"] else "?")}
            if probes.get("public-api-only", {}).get("status") == "survived":
                fin["note"] = ("killed only through a test of a private function; the public "
                               "verify2() tests do not distinguish this mutant")
        elif e["status"] == "not_applicable":
            fin = {"class": "not_applicable", "reason": e["locate"]}
        elif short in named:
            fin = {"class": named[short]["class"], "reason": named[short]["reason"]}
        else:
            fin = {"class": "open", "reason": "no argument given and no test kills it"}
        fin["stages_checked"] = stages
        if not stages:
            fin = {"class": "not_checked" if e["status"] != "not_applicable" else fin["class"],
                   "reason": fin["reason"] if e["status"] == "not_applicable"
                   else "no test run was made against this survivor",
                   "stages_checked": []}
        e["final"] = fin
        key = (e["file"], fin["class"])
        summary[key] = summary.get(key, 0) + 1
    state["summary"] = {f"{f}|{c}": n for (f, c), n in sorted(summary.items())}
    out.write_text(json.dumps(state, indent=1) + "\n")
    for k, v in state["summary"].items():
        print(k, v)
    return 0


def main() -> int:
    if len(sys.argv) == 4 and sys.argv[1] == "--worker":
        return worker_main(sys.argv[2], sys.argv[3])
    if len(sys.argv) == 4 and sys.argv[1] == "--finalize":
        return finalize(sys.argv[2], sys.argv[3])
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--mutation-json", default=str(ROOT / "results" / "mutation.json"))
    ap.add_argument("--section", default="after")
    ap.add_argument("--files", nargs="+", default=["sirb/verify2.py", "sirb/assign.py"])
    ap.add_argument("--tests", required=True, help="pytest arguments (shell-split)")
    ap.add_argument("--label", required=True, help="name of this stage, e.g. old, new, slow")
    ap.add_argument("--out", default=str(ROOT / "results" / "mutation_phase6_triage.json"))
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--mode", choices=["worker", "process"], default="worker",
                    help="worker: persistent interpreters swap function code (fast); "
                         "process: one pytest process per mutant on a temporary package copy")
    ap.add_argument("--python", default=sys.executable)
    ap.add_argument("--workdir", default=tempfile.gettempdir())
    ap.add_argument("--match", default=None, help="regex on the mutant name: run only these")
    ap.add_argument("--limit", type=int, default=None)
    ap.add_argument("--rerun-statuses", nargs="+", default=["pending", "survived", "timeout"])
    ap.add_argument("--probe", default=None,
                    help="store the result under entry['probes'][PROBE] and leave the status alone "
                         "(use with --rerun-statuses killed to ask which kills a test subset keeps)")
    ap.add_argument("--skip-baseline", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="only report how survivors locate")
    a = ap.parse_args()

    out = Path(a.out)
    state = load_state(out)
    by_name = {e["mutant"]: e for e in state["entries"]}
    recorded = json.loads(Path(a.mutation_json).read_text())[a.section]
    base = Path(a.workdir)
    base.mkdir(parents=True, exist_ok=True)
    tests = []
    for arg in shlex.split(a.tests, posix=True):
        # positional test paths are made absolute (the working directory may differ);
        # options such as --deselect keep their rootdir-relative node ids
        head, sep, tail = arg.partition("::")
        if not arg.startswith("-") and (ROOT / head).exists():
            arg = str(ROOT / head) + sep + tail
        tests.append(arg)

    todo = []
    for rel in a.files:
        surv = [dict(s) for s in recorded[rel]["survivors"]]
        mut = mutate(ROOT / rel, surv)
        for s in surv:
            e = by_name.get(s["mutant"])
            text, note = mut[s["mutant"]]
            if e is None:
                e = {"mutant": s["mutant"], "file": rel, "change": s["change"],
                     "old_class": s["class"], "old_reason": s["reason"], "locate": note,
                     "status": "pending" if text else "not_applicable", "tests": [], "history": []}
                state["entries"].append(e)
                by_name[e["mutant"]] = e
            if e["status"] in a.rerun_statuses and text:
                if a.match is None or re.search(a.match, e["mutant"]):
                    todo.append((e, text))
    if a.dry_run:
        for e in state["entries"]:
            print(f"{e['mutant'].split('.')[-1]:48s} {e['status']:15s} {e['locate']}")
        return 0
    if a.limit:
        todo = todo[: a.limit]
    print(f"{len(todo)} mutants to run, stage '{a.label}', mode {a.mode}, selection: {tests}",
          flush=True)

    if a.mode == "worker":
        counts = run_workers(a, todo, tests, state, out) if todo else {}
        out.write_text(json.dumps(state, indent=1) + "\n")
        print(counts)
        return 0

    # process mode: the selection must pass on the unmutated copy, and the copy is what is imported
    if todo and not a.skip_baseline:
        w0 = make_copy(base, "", None)
        try:
            check_import(a.python, w0)
            st, killer, secs, log = run_tests(a.python, w0, tests, a.timeout * 3)
            print(f"baseline: {st} in {secs:.0f}s", flush=True)
            if st != "survived":
                print(log[-3000:])
                raise SystemExit("the selection does not pass on the unmutated code")
        finally:
            shutil.rmtree(w0, ignore_errors=True)

    def work(item):
        e, text = item
        w = make_copy(base, e["file"], text)
        try:
            st, killer, secs, log = run_tests(a.python, w, tests, a.timeout)
        finally:
            shutil.rmtree(w, ignore_errors=True)
        rec = {"stage": a.label, "status": st, "seconds": round(secs, 1), "tests": a.tests}
        if killer:
            rec["killed_by"] = killer
        if st == "error":
            rec["log_tail"] = log[-400:]
        with _PRINT:
            if a.probe:
                e.setdefault("probes", {})[a.probe] = {k: v for k, v in rec.items() if k != "stage"}
            else:
                e["history"].append(rec)
                e["status"] = st
                e["tests"] = [killer] if killer else []
            print(f"{e['mutant'].split('.')[-1]:50s} {st:9s} {secs:5.0f}s {killer or ''}",
                  flush=True)
            out.write_text(json.dumps(state, indent=1) + "\n")
        return st

    with ThreadPoolExecutor(max_workers=max(1, a.jobs)) as pool:
        results = list(pool.map(work, todo))
    for k in ("killed", "survived", "timeout", "error"):
        print(f"{k}: {results.count(k)}")
    out.write_text(json.dumps(state, indent=1) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
