"""Viewer rendering benchmark: 1000 synthetic drones, two panes, software WebGL (swiftshader).

Builds the deterministic 1000 drone dataset (scripts/build_viewer_bench_data.py, cached in
results/bench_data/), loads viewer/index.html?bench=1 in headless Chromium, plays the show in a loop and
reports, separately, the median and p95 of
  - requestAnimationFrame interval (what a user sees),
  - the JS update step (instance matrices, colours, readouts for both panes),
  - the render submission (renderer.render of both panes, CPU side),
and writes results/viewer_bench.json. Run on an idle machine; the numbers depend on the CPU.

    nice -n 10 python scripts/viewer_bench.py [--seconds 20] [--warmup 3] [--width 1440] [--height 900]
"""

from __future__ import annotations

import argparse
import functools
import http.server
import json
import os
import platform
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _viewer_pw import CHROMIUM_ARGS, VIEWER  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "results" / "bench_data"


class Handler(http.server.SimpleHTTPRequestHandler):
    """Serves viewer/ and maps /bench/* to the generated dataset."""

    def translate_path(self, path):
        p = path.split("?", 1)[0]
        if p.startswith("/bench/"):
            return str(DATA / p[len("/bench/"):])
        return super().translate_path(path)

    def log_message(self, *a):
        pass


def pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round(q * (len(xs) - 1))))]


def stats(xs):
    return {"n": len(xs), "median_ms": round(statistics.median(xs), 3), "p95_ms": round(pct(xs, 0.95), 3),
            "mean_ms": round(statistics.fmean(xs), 3), "max_ms": round(max(xs), 3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--warmup", type=float, default=3.0)
    ap.add_argument("--width", type=int, default=1440)
    ap.add_argument("--height", type=int, default=900)
    ap.add_argument("--out", default="results/viewer_bench.json")
    ap.add_argument("--rebuild", action="store_true")
    args = ap.parse_args()
    if args.rebuild or not (DATA / "index.json").exists():
        subprocess.run([sys.executable, str(ROOT / "scripts" / "build_viewer_bench_data.py"), str(DATA)], check=True,
                       cwd=ROOT, env={**os.environ, "PYTHONPATH": str(ROOT / "scripts")})
    idx = json.loads((DATA / "index.json").read_text())["shows"][0]
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), functools.partial(Handler, directory=str(VIEWER)))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    from playwright.sync_api import sync_playwright
    with sync_playwright() as p:
        b = p.chromium.launch(headless=True, args=CHROMIUM_ARGS)
        ctx = b.new_context(viewport={"width": args.width, "height": args.height}, bypass_csp=True)
        page = ctx.new_page()
        errs = []
        page.on("pageerror", lambda e: errs.append(str(e)))
        page.goto(f"{base}/index.html?bench=1")
        page.wait_for_function("window.__bench && window.__bench.update.length > 0", timeout=180000)
        gl = page.evaluate("""() => { const c = document.createElement('canvas'); const g = c.getContext('webgl2') || c.getContext('webgl');
            const e = g && g.getExtension('WEBGL_debug_renderer_info'); return e ? g.getParameter(e.UNMASKED_RENDERER_WEBGL) : 'unknown'; }""")
        time.sleep(args.warmup)
        page.evaluate("for (const k of Object.keys(window.__bench)) window.__bench[k].length = 0")
        t0 = time.time()
        time.sleep(args.seconds)
        wall = time.time() - t0
        d = page.evaluate("window.__bench")
        version = b.version
        b.close()
    srv.shutdown()
    raf, upd, ren, tot = d["raf"], d["update"], d["render"], d["total"]
    res = {"what": "viewer frame timing, two panes, software WebGL", "drones": idx["n"], "frames_in_show": idx["frames"],
           "dataset": {"synthetic": True, "verdict": idx["verdict"], "min_separation_m": idx["min_separation_m"],
                       "d_min_m": idx["d_min_m"], "fleet_sha256": idx["fleet_sha256"]},
           "viewport": [args.width, args.height], "duration_s": round(wall, 1),
           "raf_interval": stats(raf), "js_update_step": stats(upd), "render_submit": stats(ren), "update_plus_render_js": stats(tot),
           "raf_fps_median": round(1000 / statistics.median(raf), 2), "holds_30fps": statistics.median(raf) <= 1000 / 30 and pct(raf, 0.95) <= 1000 / 20,
           "js_update_p95_under_16ms": pct(upd, 0.95) < 16.0,
           "environment": {"chromium": version, "gl_renderer": gl, "cpu_count": os.cpu_count(), "platform": platform.platform(),
                           "args": CHROMIUM_ARGS, "load_avg_1m": os.getloadavg()[0], "page_errors": errs},
           "note": "js_update_step covers both panes' update() (matrices, colours, readouts); render_submit is the CPU time of "
                   "both renderer.render calls (GPU/software raster work may complete later and shows up in raf_interval)."}
    out = ROOT / args.out
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(res, indent=1))
    print(json.dumps({k: res[k] for k in ("raf_interval", "js_update_step", "render_submit", "raf_fps_median")}, indent=1))


if __name__ == "__main__":
    main()
