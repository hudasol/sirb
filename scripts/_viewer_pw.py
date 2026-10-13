"""Shared helpers for the viewer test scripts: a quiet local static server and a Chromium launcher
configured for software WebGL (no GPU needed). Playwright and its Chromium must already be installed
(this repo never runs `playwright install`)."""

from __future__ import annotations

import contextlib
import functools
import http.server
import threading
from pathlib import Path

VIEWER = Path(__file__).resolve().parent.parent / "viewer"
CHROMIUM_ARGS = ["--use-gl=swiftshader", "--enable-unsafe-swiftshader", "--ignore-gpu-blocklist"]


class _Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass


@contextlib.contextmanager
def serve(directory: Path = VIEWER):
    handler = functools.partial(_Quiet, directory=str(directory))
    srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    try:
        yield f"http://127.0.0.1:{srv.server_address[1]}"
    finally:
        srv.shutdown()
        srv.server_close()


@contextlib.contextmanager
def browser():
    from playwright.sync_api import sync_playwright
    with sync_playwright() as p:
        b = p.chromium.launch(headless=True, args=CHROMIUM_ARGS)
        try:
            yield b
        finally:
            b.close()


def prefs_script(lang: str, theme: str) -> str:
    return (f"try{{localStorage.setItem('sirb.viewer.lang','{lang}');"
            f"localStorage.setItem('sirb.viewer.theme','{theme}')}}catch(e){{}}")


def wait_loaded(page, scenario: str | None = "motor_failure", timeout: int = 90000):
    """Wait until a show and the scenario data are rendered (frames counter advances)."""
    page.wait_for_function("window.__sirb && window.__sirb.frames > 0 && "
                           "!document.querySelector('.overlay:not(:empty)')", timeout=timeout)
    if scenario:
        page.select_option("#scenSelect", scenario)
        page.wait_for_function("!document.querySelector('.overlay:not(:empty)') && "
                               "document.querySelector('#scenBody h3')", timeout=timeout)
        page.evaluate("window.__sirbForce()")
        page.wait_for_timeout(300)
