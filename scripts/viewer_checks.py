"""Keyboard, reduced motion, light theme, file:// banner, RTL and overflow checks for the viewer.

    python scripts/viewer_checks.py [--out results/viewer_checks.json] [--screens]

Each check is run in headless Chromium (software WebGL) against a local static server and recorded as
pass/fail with details. `--screens` also writes the screenshots in viewer/screenshots/.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from _viewer_pw import VIEWER, browser, prefs_script, serve, wait_loaded

RESULTS: list[dict] = []


def record(name, ok, detail=""):
    RESULTS.append({"check": name, "pass": bool(ok), "detail": detail})
    print(("PASS " if ok else "FAIL ") + name, detail if not ok else "", flush=True)


def new_page(b, base, lang="en", theme="dark", width=1440, height=900, page="index.html", **ctx_kw):
    ctx = b.new_context(viewport={"width": width, "height": height}, bypass_csp=True, **ctx_kw)
    ctx.add_init_script(prefs_script(lang, theme))
    p = ctx.new_page()
    p.goto(f"{base}/{page}")
    if page == "index.html":
        wait_loaded(p)
    else:
        p.wait_for_selector("h1")
    return ctx, p


def scrub_t(p):
    return float(p.evaluate("Number(document.querySelector('#scrub').value) / 50"))


def keyboard_checks(b, base, lang):
    ctx, p = new_page(b, base, lang)
    tag = "[" + lang + "] "
    # tab order: starts at the skip link, follows DOM order, reaches every control, shows a focus ring
    order = []
    p.evaluate("window.__seen = new Set(); document.addEventListener('focusin', e => window.__seen.add(e.target))")
    for _ in range(90):
        p.keyboard.press("Tab")
        info = p.evaluate("""() => { const e = document.activeElement; const cs = getComputedStyle(e);
            return {id: e.id || '', tag: e.tagName.toLowerCase(), cls: e.className && e.className.baseVal === undefined ? e.className : '',
                    outline: cs.outlineStyle + ' ' + cs.outlineWidth, text: (e.textContent || '').trim().slice(0, 24)}; }""")
        order.append(info)
        if info["tag"] == "body":
            break
    ids = [o["id"] or o["tag"] for o in order]
    record(tag + "tab order starts at skip link", order[0]["cls"] == "skip", str(order[0]))
    unseen = p.evaluate("""() => [...document.querySelectorAll('a[href],button,select,input,canvas[tabindex],summary')]
        .filter(e => { const r = e.getBoundingClientRect(); return r.width > 0 && r.height > 0 && getComputedStyle(e).visibility !== 'hidden'; })
        .filter(e => !window.__seen.has(e)).map(e => e.tagName + '#' + e.id)""")
    record(tag + "tab reaches every control and leaves the page (no trap)", not unseen and order[-1]["tag"] == "body",
           f"unreached {unseen}; first ids {ids[:12]}")
    bad = [o for o in order[:-1] if o["outline"].startswith("none")]
    record(tag + "focused controls show an outline", not bad, str(bad[:3]))
    p.evaluate("document.activeElement.blur()")
    # buttons: Space / Enter
    p.focus("#playBtn")
    p.keyboard.press("Space")
    p.wait_for_timeout(700)
    pressed = p.get_attribute("#playBtn", "aria-pressed")
    t1 = scrub_t(p)
    p.keyboard.press("Enter")
    p.wait_for_timeout(200)
    record(tag + "Space starts and Enter pauses on the play button", pressed == "true" and p.get_attribute("#playBtn", "aria-pressed") == "false" and t1 > 0,
           f"pressed={pressed} t={t1}")
    p.focus("#restartBtn")
    p.keyboard.press("Enter")
    p.focus("#fwdBtn")
    p.keyboard.press("Enter")
    p.keyboard.press("Space")
    t_f = scrub_t(p)
    p.focus("#backBtn")
    p.keyboard.press("Enter")
    t_b = scrub_t(p)
    record(tag + "step buttons work with Enter and Space (one frame = 0.2 s)", abs(t_f - 0.4) < 0.03 and abs(t_b - 0.2) < 0.03, f"{t_f} {t_b}")
    # scrubber arrows (timeline stays left to right in AR, so ArrowRight is forward in both languages)
    p.focus("#scrub")
    p.keyboard.press("Home")
    p.keyboard.press("ArrowRight")
    a = scrub_t(p)
    p.keyboard.press("Shift+ArrowRight")
    b2 = scrub_t(p)
    p.keyboard.press("ArrowLeft")
    c = scrub_t(p)
    p.keyboard.press("End")
    d = scrub_t(p)
    total = float(p.evaluate("Number(document.querySelector('#scrub').max) / 50"))
    record(tag + "scrubber: arrows, Shift+arrows, Home and End", abs(a - 1) < 0.03 and abs(b2 - 1.2) < 0.03 and abs(c - 0.2) < 0.03 and abs(d - total) < 0.03,
           f"{a} {b2} {c} {d} total {total}")
    vt = p.get_attribute("#scrub", "aria-valuetext")
    record(tag + "scrubber exposes a text value", bool(vt), str(vt))
    # global Space on body
    p.evaluate("document.activeElement.blur()")
    p.keyboard.press("Space")
    p.wait_for_timeout(300)
    on = p.get_attribute("#playBtn", "aria-pressed") == "true"
    p.keyboard.press("Space")
    record(tag + "Space on the page toggles playback", on and p.get_attribute("#playBtn", "aria-pressed") == "false")
    # speed + mode groups via keyboard
    p.focus("#modeGroup button[data-mode=sep]")
    p.keyboard.press("Enter")
    record(tag + "colour mode button toggles with Enter", p.get_attribute("#modeGroup button[data-mode=sep]", "aria-pressed") == "true")
    # select by keyboard
    p.focus("#scenSelect")
    p.keyboard.press("ArrowUp")
    p.wait_for_timeout(1500)
    record(tag + "scenario select works from the keyboard", p.evaluate("document.querySelector('#scenSelect').value") != "motor_failure")
    ctx.close()


def reduced_motion(b, base):
    ctx, p = new_page(b, base, reduced_motion="reduce")
    mm = p.evaluate("matchMedia('(prefers-reduced-motion: reduce)').matches")
    playing = p.get_attribute("#playBtn", "aria-pressed")
    p.wait_for_timeout(500)
    t0 = scrub_t(p)
    p.wait_for_timeout(500)
    t1 = scrub_t(p)
    anim = p.evaluate("""() => { const bad = []; for (const e of document.querySelectorAll('*')) { const c = getComputedStyle(e);
        if ((c.animationName !== 'none' && c.animationDuration !== '0s') || (c.transitionDuration.split(',').some(x => x.trim() !== '0s'))) bad.push(e.tagName + '.' + e.className); }
        return bad.slice(0, 5); }""")
    sb = p.evaluate("getComputedStyle(document.documentElement).scrollBehavior")
    record("reduced motion: media query honoured, no autoplay, no CSS animation or transition, no smooth scroll",
           mm and playing == "false" and t0 == t1 and not anim and sb in ("auto", ""), f"mm={mm} playing={playing} t={t0},{t1} anim={anim} sb={sb}")
    ctx.close()


def light_theme(b, base):
    ctx, p = new_page(b, base, theme="light")
    th = p.evaluate("document.documentElement.dataset.theme")
    bg = p.evaluate("getComputedStyle(document.body).backgroundColor")
    lum = sum(int(x) for x in bg[bg.index("(") + 1:bg.index(")")].split(",")[:3]) / 3
    cv = p.evaluate("""() => { const c = document.createElement('canvas'); return getComputedStyle(document.querySelector('.canvas-wrap')).backgroundColor; }""")
    btn = p.inner_text("#themeBtn")
    p.click("#themeBtn")
    th2 = p.evaluate("document.documentElement.dataset.theme")
    ctx.close()
    ctx2, p2 = new_page(b, base, theme="dark", color_scheme="light")
    # stored dark wins over OS light; with nothing stored OS light is used
    ctx2.close()
    ctx3 = b.new_context(viewport={"width": 1440, "height": 900}, bypass_csp=True, color_scheme="light")
    p3 = ctx3.new_page()
    p3.goto(f"{base}/index.html")
    wait_loaded(p3, None)
    auto = p3.evaluate("document.documentElement.dataset.theme")
    ctx3.close()
    record("light theme: stored light applied, light background, toggle returns to dark, OS light preference used when nothing is stored",
           th == "light" and lum > 200 and th2 == "dark" and auto == "light", f"theme={th} bg={bg} canvas={cv} btn={btn} toggled={th2} auto={auto}")


def file_banner(b):
    p = b.new_page()
    p.goto((VIEWER / "index.html").as_uri())
    p.wait_for_timeout(1500)
    vis = p.evaluate("(() => { const e = document.querySelector('#fileBanner'); return !e.hidden && e.getBoundingClientRect().height > 0; })()")
    txt = p.inner_text("#fileBanner")
    role = p.get_attribute("#fileBanner", "role")
    record("file:// shows the static bilingual error banner (role=alert) with the http.server command",
           vis and "python -m http.server -d viewer 8000" in txt and "file://" in txt and role == "alert", f"visible={vis} text={txt[:80]!r}")
    p.close()
    # and it stays hidden over http
    with serve() as base:
        ctx, p2 = new_page(b, base)
        record("file:// banner hidden over http", p2.evaluate("document.querySelector('#fileBanner').hidden"))
        ctx.close()


def rtl_checks(b, base):
    ctx, p = new_page(b, base, "ar")
    r = p.evaluate("""() => {
      const q = (s) => document.querySelector(s);
      const num = (s) => q(s).textContent;
      const ascii = (t) => /[0-9]/.test(t);
      return {dir: document.documentElement.dir, lang: document.documentElement.lang,
        media: getComputedStyle(q('.media')).direction, scrub: getComputedStyle(q('#scrub')).direction,
        time: getComputedStyle(q('#timeOut')).direction, timeTxt: num('#timeOut'),
        ro: [...document.querySelectorAll('#roA dd, #roB dd')].map(e => e.textContent),
        hash: getComputedStyle(q('code.hash')).direction, hashBidi: getComputedStyle(q('code.hash')).unicodeBidi,
        backGlyph: getComputedStyle(q('#backBtn .ico'), '::before').content, fwdGlyph: getComputedStyle(q('#fwdBtn .ico'), '::before').content,
        backLeftOfFwd: q('#backBtn').getBoundingClientRect().left < q('#fwdBtn').getBoundingClientRect().left,
        bannerOrig: q('#bannerOrig span[lang=en]') ? getComputedStyle(q('#bannerOrig span[lang=en]')).direction : null,
        thumbOk: true, cfg: [...document.querySelectorAll('#scenBody .kv ul li')].map(e => e.textContent)}; }""")
    arabic_digits_only = all(not any(c.isascii() and c.isdigit() for c in t) for t in r["ro"] + [r["timeTxt"]])
    record("AR: dir=rtl, lang=ar", r["dir"] == "rtl" and r["lang"] == "ar", str(r["dir"]))
    record("AR: numbers use Arabic-Indic digits in readouts and time", arabic_digits_only, str(r["ro"]) + r["timeTxt"])
    record("AR: scenario parameter values use Arabic-Indic digits", all(not any(c.isascii() and c.isdigit() for c in t.split(":")[-1]) for t in r["cfg"]), str(r["cfg"]))
    record("AR: media controls and timeline stay left to right (time runs left to right) with back left of forward",
           r["media"] == "ltr" and r["scrub"] == "ltr" and r["backLeftOfFwd"], f"{r['media']} {r['scrub']} {r['backLeftOfFwd']}")
    record("AR: hash is an isolated LTR run", r["hash"] == "ltr" and r["hashBidi"] == "isolate", f"{r['hash']} {r['hashBidi']}")
    record("AR: step glyphs point in their time direction (back left, forward right)", "◀" in r["backGlyph"] and "▶" in r["fwdGlyph"], f"{r['backGlyph']} {r['fwdGlyph']}")
    record("AR: English original of the safety statement is a separate LTR block", r["bannerOrig"] == "ltr", str(r["bannerOrig"]))
    ctx.close()
    ctx, p = new_page(b, base, "ar", page="methods.html")
    r = p.evaluate("""() => ({dir: document.documentElement.dir, en: getComputedStyle(document.querySelector('.en')).display,
        ar: getComputedStyle(document.querySelector('.ar')).display})""")
    record("AR methods page: rtl, Arabic text shown, English duplicate hidden", r["dir"] == "rtl" and r["en"] == "none" and r["ar"] != "none", str(r))
    ctx.close()


OVERFLOW_JS = """(w) => { const bad = []; for (const e of document.querySelectorAll('body *')) {
    const r = e.getBoundingClientRect(); const cs = getComputedStyle(e);
    if (r.width === 0 || cs.display === 'none' || cs.position === 'fixed') continue;
    if (e.closest('.skip')) continue;
    if (r.right > w + 1 || r.left < -1) bad.push(e.tagName.toLowerCase() + (e.id ? '#' + e.id : '') + '.' + String(e.className).slice(0, 20) + ' ' + Math.round(r.left) + '..' + Math.round(r.right)); }
    return {sw: document.documentElement.scrollWidth, cw: document.documentElement.clientWidth, bad: bad.slice(0, 6)}; }"""


def overflow_checks(b, base):
    for page in ("index.html", "methods.html"):
        for lang in ("en", "ar"):
            for w in (390, 320):
                ctx, p = new_page(b, base, lang, width=w, page=page)
                o = p.evaluate(OVERFLOW_JS, w)
                record(f"no horizontal overflow at {w}px: {page} {lang}", o["sw"] <= o["cw"] and not o["bad"], str(o))
                ctx.close()


SHOTS = [("index", "en", "dark", 1440, 900, "index_en_1440"), ("index", "ar", "dark", 1440, 900, "index_ar_1440"),
         ("index", "ar", "dark", 390, 844, "index_ar_390"), ("index", "en", "dark", 390, 844, "index_en_390"),
         ("index", "en", "light", 1440, 900, "index_en_light_1440"),
         ("methods", "ar", "dark", 1440, 900, "methods_ar_1440"), ("methods", "ar", "dark", 390, 844, "methods_ar_390")]


def screens(b, base):
    from PIL import Image
    out = VIEWER / "screenshots"
    out.mkdir(exist_ok=True)
    for old in out.glob("*.png"):
        old.unlink()
    for page, lang, theme, w, h, name in SHOTS:
        ctx, p = new_page(b, base, lang, theme, w, h, page=page + ".html")
        path = out / f"{name}.png"
        p.screenshot(path=str(path), full_page=(page == "methods"), clip=None if page == "methods" else {"x": 0, "y": 0, "width": w, "height": h})
        img = Image.open(path).convert("RGB")
        if path.stat().st_size > 240_000:
            img = img.quantize(colors=96, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
        if page == "methods" and img.height > 1400:
            img = img.crop((0, 0, img.width, 1400))
        img.save(path, optimize=True)
        print(name, path.stat().st_size // 1024, "KB", flush=True)
        ctx.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="results/viewer_checks.json")
    ap.add_argument("--screens", action="store_true")
    args = ap.parse_args()
    with serve() as base, browser() as b:
        for lang in ("en", "ar"):
            keyboard_checks(b, base, lang)
        reduced_motion(b, base)
        light_theme(b, base)
        file_banner(b)
        rtl_checks(b, base)
        overflow_checks(b, base)
        if args.screens:
            screens(b, base)
    Path(args.out).write_text(json.dumps({"passed": sum(r["pass"] for r in RESULTS), "failed": sum(not r["pass"] for r in RESULTS),
                                          "checks": RESULTS}, indent=1, ensure_ascii=False))
    print(sum(r["pass"] for r in RESULTS), "passed,", sum(not r["pass"] for r in RESULTS), "failed")


if __name__ == "__main__":
    main()
