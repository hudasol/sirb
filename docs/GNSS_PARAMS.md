# GNSS model parameter file (version 1)

**Status: no external GNSS study, dataset or repository is coupled to Sirb's model.** Three files ship.
`shows/gnss_assumed_default.json` reproduces the defaults of `StressConfig`. Since phase 6,
`shows/gnss_faa_sps_standalone.json` takes the bias magnitude from public FAA accuracy statistics
(`docs/GNSS_SOURCED.md`); the source gives the absolute error of one receiver, not the relative
error between drones the model needs, so every value in both files is flagged `assumed: true`.
`shows/gnss_relative_literature.json` (phase 6, stream D) anchors the bias magnitude to a published
value for the error left after common mode cancellation (`docs/GNSS_SOURCED.md`); it is a literature
value for different hardware, so its flags also stay `assumed: true`. Nothing in any of the three
files was measured by this project.

## Why a file format

The GNSS error model in `sirb/disturbance.py` (`gnss_error`) is driven by seven fields of
`StressConfig`. Before this format, a number could reach a stress run without anyone recording where
it came from. The file puts the number, its unit, a provenance block and an explicit assumed flag
next to each other, and the loader refuses a file that leaves any of them out.

Code: `sirb/gnss_params.py` (`load`, `save`, `validation_errors`, `from_dict`, `to_stress_config`,
`from_stress_config`). Machine-readable schema: `sirb/schemas/gnss_params.schema.json` (JSON Schema
draft 2020-12). The Python validator is the authority; the schema mirrors it for other tools and
cannot express every rule (ISO date validity, unique region names, `end_s > start_s`, bounding box
ordering). `jsonschema` is not a dependency of Sirb; a test uses it only if it is installed.

## Format

```json
{
  "schema_version": 1,
  "model": "sirb-gnss-gauss-markov",
  "provenance": { "source_name": "...", "url": "...", "licence": "...",
                  "retrieval_date": "YYYY-MM-DD", "derived_by": "...", "notes": "optional" },
  "parameters": { "<name>": { "value": 0.0, "unit": "m", "assumed": true } },
  "spatial":  { "region_scale_factors": [ { "name": "...", "scale": 1.0, "assumed": true,
                                            "bbox_xy_m": [xmin, ymin, xmax, ymax] } ] },
  "temporal": { "window": { "start_s": 0.0, "end_s": 60.0, "assumed": true } }
}
```

`spatial` and `temporal` are optional. Unknown keys anywhere are errors, duplicate JSON keys and
`NaN`/`Infinity` are errors, and `schema_version` must be exactly 1.

| Parameter | Unit | Range | `StressConfig` field | Role in `gnss_error` |
|---|---|---|---|---|
| `white_sigma` | m | >= 0 | `gnss_white_sigma` | per-axis white noise on every drone |
| `degraded_fraction` | 1 | 0 to 1 | `gnss_deg_fraction` | share of drones that get the correlated bias |
| `bias_sigma` | m | >= 0 | `gnss_bias_sigma` | std of the first order Gauss-Markov bias, per axis |
| `bias_tau` | s | > 0 | `gnss_bias_tau` | correlation time of that bias |
| `dropout_fraction` | 1 | 0 to 1 | `gnss_loss_fraction` | share of drones with one dropout window |
| `dropout_duration` | s | > 0 | `gnss_loss_s` | length of the dropout window |
| `dead_reckoning_rate` | m/sqrt(s) | >= 0 | `gnss_dead_reckoning_rate` | random walk rate during dropout |

Units are written in the file and must match exactly; a value in centimetres is rejected rather than
converted. `assumed` is required on every parameter and must be a JSON boolean; omitting it is an
error, so "unknown status" cannot be expressed.

**Provenance** (all five mandatory, non-empty): `source_name`; `url` (an http(s) URL, or a relative
repository path when the source is a document in this repository); `licence`; `retrieval_date`
(ISO `YYYY-MM-DD`; for hand-set assumptions, the date the file was written); `derived_by` (what
produced the numbers from the source: a script name and version, a person, or the code that wrote
the file). `notes` is optional. The loader checks form, not truth.

**Optional blocks.** `spatial.region_scale_factors` are dimensionless multipliers per named region
(optionally a bounding box in the show frame, metres); `temporal.window` is a window in show
seconds. They exist so that an importer for regional or time varying interference has somewhere to
put its output. **`StressConfig` and `gnss_error` cannot use them.** `to_stress_config` therefore
raises if a file carries either block, unless the caller passes `ignore_unsupported=True` and so
drops them knowingly. Using them needs a code change to `gnss_error` and a new experiment, and
neither exists.

## Conversion

- `to_stress_config(params, base=None, ignore_unsupported=False)` returns `base` (default
  `StressConfig()`) with only the seven GNSS fields replaced; wind, tracking and event fields are
  untouched.
- `from_stress_config(cfg, provenance, assumed)` captures the seven fields of a config. `assumed`
  has to be stated (one boolean for all, or a flag per parameter).
- `shows/gnss_assumed_default.json` was written with `from_stress_config(StressConfig(), ...)`.
  `tests/test_gnss_params.py` asserts that loading it gives a `StressConfig` equal to the default one,
  so the file and the code cannot disagree. Its provenance cites `docs/METHODS.md` (section 5) and
  lists the licence of this repository; the reasoning behind each value is in `docs/RESEARCH.md`
  section 3 (all position-domain values there are marked ASSUMPTION).

## How a future importer would fill it

1. Take a dataset or study that is public, has an explicit licence permitting reuse, and states
   position-domain error statistics (not only counts of events). Record its name, URL, licence and
   the retrieval date.
2. Compute each parameter by a script kept under version control; put that script's name and commit in
   `derived_by`. Convert units in the script, not in the file.
3. Mark `assumed: false` only for a number that the script derived from the source. Anything
   chosen by judgement, and any mapping between the source's quantity and the model's (for example
   a pseudorange statistic into position error), stays `assumed: true` and the mapping is described in
   `notes`.
4. Regional or time varying statistics go into `spatial` or `temporal` only once `gnss_error` can use
   them; until then they would have to be dropped via `ignore_unsupported`.
5. Load with `sirb.gnss_params.load`, convert, run the stress experiment, and report the provenance
   block with the result.

## What was looked at for coupling (read only, 2026-10-12)

- `hudasol/marsad`, read through the GitHub connector: the repository is **public**. `pyproject.toml`
  and the README state **Apache-2.0**; a `LICENSE` file is present (its text was not opened).
  The README says its results come from measurement-level simulation, that nothing was run on real
  interference data, and that it detects and scores interference rather than publishing position error
  statistics. Its documented outputs (trust reports, a hex-binned interference confidence map as
  GeoJSON) are not the parameters this format needs, and the README describes no dataset of measured
  error. **Nothing from it is used.** A repository file listing was not walked further, and its `src/`,
  `docs/` and `scripts/` contents were not read.
- The Tashwish / Gulf GNSS Interference Observatory repository: searches of the `hudasol` account
  for "tashwish", "interference", "jamming", "observatory" and "gulf" returned only `marsad`. It
  is therefore either private, named differently, or not under that account as seen by this
  connector. **Nothing from it is used**, and no claim is made about what it contains or licences.
- Neither repository was modified.

If a licensed public output with position-domain statistics appears later, cite the exact file,
commit and licence in the provenance block, keep `assumed: true` for everything the file does not
directly give, and do not describe the result as validated against real flights.

## Limits

The format cannot say whether a value is right, only that its origin was declared. Per-drone errors
in `gnss_error` stay independent, so a file cannot describe fleet-correlated interference (see
`docs/HAZARDS.md`, H9, H10 and H18).
