# Web viewer (M7)

Static site in `viewer/`: no build step, no CDN. `index.html` + `app.js` (ES modules), `i18n.js` (EN/AR strings), `style.css`, `methods.html` (how every number is computed, bilingual), `vendor/` (three.js 0.170.0 and OrbitControls, MIT, license in `vendor/LICENSE`), `data/` (generated).

## Run

```
python scripts/build_viewer_data.py      # only to regenerate viewer/data/
python -m http.server -d viewer 8000     # then open http://localhost:8000/
```

It must be served over HTTP; `file://` cannot fetch the binary data (the page shows an error with this hint).

## What it shows

- Left: nominal verified plan. Right: a chosen stressed scenario (fixed-seed simulation under assumed models). One shared timeline and linked orbit cameras.
- Colour by height or by nearest-neighbour separation (red < d_min, amber < 1.25 d_min, green). Event drones get wireframe diamonds from their event time.
- Certificate panel (verdict, exact min separation, d_min, short SHA-256 with copy, recomputation check (the same verifier code re-run, hence not independent), guarantees and non-guarantees) and a permanent safety banner.
- Every analysis number links to a section of `methods.html`.
- EN/AR toggle (`dir=rtl`, logical CSS, Arabic-Indic digits), remembered in `localStorage` (also passed as `?lang=` to the methods page). Dark by default unless the OS prefers light; manual toggle.

## Limits

Readouts are nearest-neighbour distances sampled every 0.2 s and can miss minima between frames; the exact verifier figure is the authoritative one for the nominal plan. Stressed results are simulation under assumed models, never guarantees. Drones are drawn enlarged (0.9 m). Simulation only; not flight certified.
