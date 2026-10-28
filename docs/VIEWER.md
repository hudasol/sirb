# Web viewer (M7)

Static site in `viewer/`: no build step, no CDN. `index.html` + `app.js` (ES modules), `i18n.js` (EN/AR strings), `style.css`, `methods.html` (how every number is computed, bilingual), `vendor/` (three.js 0.170.0 and OrbitControls, MIT, license in `vendor/LICENSE`), `data/` (generated).

## Run

```
python scripts/build_viewer_data.py      # only to regenerate viewer/data/
python -m http.server -d viewer 8000     # then open http://localhost:8000/
```

It must be served over HTTP; `file://` cannot fetch the binary data (the page shows an error with this hint).

## What it shows

- Left: nominal plan (it passed the specified separation check). Right: a chosen stressed scenario (fixed-seed simulation under assumed models). One shared timeline and linked orbit cameras.
- Colour by height or by nearest-neighbour separation (red < d_min, amber < 1.25 d_min, green). Event drones get wireframe diamonds from their event time.
- Certificate panel (verdict, exact min separation, d_min, short SHA-256 with copy, recomputation check (the same verifier code re-run, hence not independent), guarantees and non-guarantees) and a permanent safety banner.
- Every analysis number links to a section of `methods.html`.
- EN/AR toggle (`dir=rtl`, logical CSS, Arabic-Indic digits), remembered in `localStorage` (also passed as `?lang=` to the methods page). Dark by default unless the OS prefers light; manual toggle.

## Accessibility, RTL and checks (goal 6)

Scripts (Playwright with the preinstalled Chromium, software WebGL, never `playwright install`):

```
python scripts/axe_check.py --label after   # axe-core 4.14.0 on index and methods, EN/AR, dark/light -> results/axe.json
python scripts/viewer_checks.py --screens   # keyboard, reduced motion, light theme, file://, RTL, overflow -> results/viewer_checks.json, viewer/screenshots/
```

axe-core (MPL-2.0) lives in `scripts/vendor/axe-core/` and is injected by the script only; it is not part of the shipped site (see `docs/DATA_SOURCES.md`). The index page is audited with a show loaded and a scenario selected. The audit contexts bypass the page CSP, because injecting a script is otherwise refused; the shipped pages keep their CSP.

**axe before and after** (8 runs: 2 pages x EN/AR x dark/light; `results/axe.json`): before, 18 rule violations (12 moderate, 6 serious; 162 nodes) and 4 incomplete checks; after, 0 violations and 0 incomplete. No colour contrast issue was reported in either theme, so none was changed. axe covers only part of WCAG (roughly the machine checkable part); it does not test reading order, focus behaviour beyond tab reachability, or screen reader output, and it cannot judge the 3D canvas.

Fixes: the safety banner lost its landmark because `role="note"` replaced the `aside` role (now a labelled complementary landmark); methods.html gained `<header>` and `<main>`; the language button keeps its visible name (an `aria-label` that did not contain the visible text moved to `title`); the step buttons draw their glyphs with CSS so the name is only the label; the section labels "3D views" and "Playback" were hardcoded English and are now translated.

**Arabic and RTL decisions.** The page is `dir=rtl` with logical CSS. The nominal pane is first in reading order, so it sits on the right in Arabic. Time is not mirrored: the timeline, and the back, play and forward group, stay left to right (`dir=ltr`), like media players in RTL interfaces, so the arrows point in the direction time moves and ArrowRight always means later. Hashes, `d_min`, file names and commands are isolated LTR runs; the English original of the safety statement is its own LTR block under an Arabic label. Numbers use Arabic-Indic digits everywhere in the interface, including scenario parameters and the seed; digits inside quoted technical identifiers and in the long Methods prose keep Latin form. Checked at 1440 and 390 px (and 320 px) in both languages for horizontal overflow.

**Checks** (`results/viewer_checks.json`, 40 of 40 pass): tab order from the skip link through every control without a trap and with a visible focus outline; Space and Enter on buttons; scrubber arrows, Shift+arrows, Home, End; global Space; `prefers-reduced-motion` (no autoplay, no CSS animation or transition; the page has no animation other than user started playback); light theme (stored, OS preference, toggle); `file://` (a classic script, `fileguard.js`, reveals a static bilingual notice, because ES modules cannot load from `file://` and previously the page stayed blank with no message); RTL and overflow checks. Screenshots are in `viewer/screenshots/`.

## Benchmark (measured; `results/viewer_bench.json`)

```
nice -n 10 python scripts/viewer_bench.py
```

Builds `results/bench_data/` (git ignored) with `scripts/build_viewer_bench_data.py`: a deterministic 1000 drone synthetic show (launch grid, sphere, landing; enlarged geofence, baseline planner). The planner output passes the exact verifier at 1000 drones (its minimum separation is recorded in `results/viewer_bench.json`), so the dataset is a valid verified show, not only a rendering load. The script then loads `index.html?bench=1` (data read from `./bench/`, per frame timings in `window.__bench`), plays the show in a loop and writes `results/viewer_bench.json` with median and p95 of the requestAnimationFrame interval, the JS update step and the render submission separately. The measurement is recorded and graded in prediction P8. It was made before the phase 6 target change and was not rerun in phase 6, so its dataset predates the snapped targets (`docs/CHANGES.md`); the frame timings do not depend on the exact positions.

## Limits

Readouts are nearest-neighbour distances sampled every 0.2 s and can miss minima between frames; the exact verifier figure is the authoritative one for the nominal plan. Stressed results are simulation under assumed models, never guarantees. Drones are drawn enlarged (0.9 m). Simulation only; not flight certified.

## Security notes (review round 6)

- No HTML injection sinks: no `innerHTML`, `outerHTML`, `insertAdjacentHTML`, `document.write`, `eval` or `new Function` in `app.js`, `i18n.js` or the pages; all text from JSON goes through `textContent`/`createTextNode`. Checked statically (test) and dynamically in headless Chromium by loading a copy of the viewer whose `index.json` and `certificate.json` had markup payloads in every string field (show name, scenario labels, config keys, event kinds, guarantees, statement): the payloads render as literal text and no script ran.
- Each page carries a `Content-Security-Policy` meta tag: `default-src 'self'`, scripts from the same origin plus the SHA-256 of the one inline script per page (so changing an inline script requires updating its hash; a test checks this), `style-src 'self' 'unsafe-inline'` (one inline `style` attribute and JS-set styles remain), `connect-src 'self'`, no objects, no `base`, no form posts. Verified in headless Chromium: the page works under it and an injected inline script is refused.
- A meta tag cannot set `frame-ancestors`, `X-Content-Type-Options` or HSTS. When the site is deployed (M9), set those as HTTP response headers on the host.
- Data files are same-origin static files; the viewer does not accept data from URLs. The `?lang=` and `#hash` inputs are validated against `en`/`ar` and used only as an id lookup.

## Results page

`viewer/results.html` lists the main results with their source file, the first 12 characters of that file's SHA-256 and the seed, the events in the stressed scenarios, and the limitations. It is generated by `scripts/make_results_page.py` from `results/*.json` and `viewer/data/index.json`; a test fails if the committed page differs from what the generator produces.
