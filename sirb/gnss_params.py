"""Versioned parameter file for the GNSS degradation model of ``sirb.disturbance``.

A file (JSON, schema ``sirb/schemas/gnss_params.schema.json``, description in
``docs/GNSS_PARAMS.md``) carries the seven GNSS fields of ``StressConfig`` together with a
mandatory provenance block and a per-parameter ``assumed`` flag, so a number can never travel
without saying where it came from. Validation is strict: unknown keys, missing provenance,
missing ``assumed`` flags, wrong units, non-finite or out-of-range values are all errors.

Optional ``spatial`` and ``temporal`` blocks are part of the format for future importers, but
``StressConfig`` has no way to use them today; ``to_stress_config`` refuses a file that carries
them unless the caller explicitly asks to ignore them. No external data source is coupled.
"""

from __future__ import annotations

import datetime as _dt
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from .disturbance import StressConfig
from .fileio import read_text_limited

SCHEMA_VERSION = 1
MODEL_ID = "sirb-gnss-gauss-markov"

# name -> (StressConfig field, unit, lower bound, lower inclusive, upper bound)
PARAMS: dict[str, tuple[str, str, float, bool, float]] = {
    "white_sigma": ("gnss_white_sigma", "m", 0.0, True, math.inf),
    "degraded_fraction": ("gnss_deg_fraction", "1", 0.0, True, 1.0),
    "bias_sigma": ("gnss_bias_sigma", "m", 0.0, True, math.inf),
    "bias_tau": ("gnss_bias_tau", "s", 0.0, False, math.inf),
    "dropout_fraction": ("gnss_loss_fraction", "1", 0.0, True, 1.0),
    "dropout_duration": ("gnss_loss_s", "s", 0.0, False, math.inf),
    "dead_reckoning_rate": ("gnss_dead_reckoning_rate", "m/sqrt(s)", 0.0, True, math.inf),
}
PROVENANCE_KEYS = ("source_name", "url", "licence", "retrieval_date", "derived_by")
PROVENANCE_OPTIONAL = ("notes",)
TOP_KEYS = {"schema_version", "model", "provenance", "parameters", "spatial", "temporal"}
TOP_REQUIRED = ("schema_version", "model", "provenance", "parameters")


class GnssParamsError(ValueError):
    """Raised with every problem found, one per line."""


@dataclass
class GnssParams:
    provenance: dict
    values: dict[str, float]
    assumed: dict[str, bool]
    spatial: dict | None = None
    temporal: dict | None = None
    schema_version: int = SCHEMA_VERSION
    model: str = MODEL_ID
    extras: dict = field(default_factory=dict)  # reserved; always empty after validation

    def to_dict(self) -> dict:
        out: dict = {
            "schema_version": self.schema_version,
            "model": self.model,
            "provenance": dict(self.provenance),
            "parameters": {n: {"value": self.values[n], "unit": PARAMS[n][1],
                               "assumed": self.assumed[n]} for n in PARAMS},
        }
        if self.spatial is not None:
            out["spatial"] = json.loads(json.dumps(self.spatial))
        if self.temporal is not None:
            out["temporal"] = json.loads(json.dumps(self.temporal))
        return out


def _is_num(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def _unknown(d: dict, allowed, where: str, errs: list[str]) -> None:
    for k in d:
        if k not in allowed:
            errs.append(f"{where}: unknown key {k!r}")


def _check_provenance(p, errs: list[str]) -> None:
    if not isinstance(p, dict):
        errs.append("provenance must be an object")
        return
    _unknown(p, set(PROVENANCE_KEYS) | set(PROVENANCE_OPTIONAL), "provenance", errs)
    for k in PROVENANCE_KEYS:
        v = p.get(k)
        if not isinstance(v, str) or not v.strip():
            errs.append(f"provenance.{k} is required and must be a non-empty string")
    if "notes" in p and not isinstance(p["notes"], str):
        errs.append("provenance.notes must be a string")
    d = p.get("retrieval_date")
    if isinstance(d, str) and d.strip():
        try:
            _dt.date.fromisoformat(d)
            if len(d) != 10:
                raise ValueError
        except ValueError:
            errs.append("provenance.retrieval_date must be an ISO date YYYY-MM-DD")
    u = p.get("url")
    if isinstance(u, str) and u.strip() and (" " in u or u.startswith("/") or "\\" in u):
        errs.append("provenance.url must be an http(s) URL or a relative repository path "
                    "without spaces")


def _check_parameters(par, errs: list[str]) -> None:
    if not isinstance(par, dict):
        errs.append("parameters must be an object")
        return
    _unknown(par, PARAMS, "parameters", errs)
    for name, (_f, unit, lo, lo_incl, hi) in PARAMS.items():
        e = par.get(name)
        if e is None:
            errs.append(f"parameters.{name} is missing")
            continue
        if not isinstance(e, dict):
            errs.append(f"parameters.{name} must be an object")
            continue
        _unknown(e, {"value", "unit", "assumed"}, f"parameters.{name}", errs)
        if "assumed" not in e:
            errs.append(f"parameters.{name}.assumed is missing (true or false is required)")
        elif not isinstance(e["assumed"], bool):
            errs.append(f"parameters.{name}.assumed must be a boolean")
        if e.get("unit") != unit:
            errs.append(f"parameters.{name}.unit must be {unit!r}, got {e.get('unit')!r}")
        v = e.get("value")
        if not _is_num(v):
            errs.append(f"parameters.{name}.value must be a finite number")
        elif v < lo or (v == lo and not lo_incl) or v > hi:
            rng = f"{'>=' if lo_incl else '>'} {lo}" + (f" and <= {hi}" if hi != math.inf else "")
            errs.append(f"parameters.{name}.value {v} out of range (must be {rng})")


def _check_spatial(s, errs: list[str]) -> None:
    if not isinstance(s, dict):
        errs.append("spatial must be an object")
        return
    _unknown(s, {"region_scale_factors"}, "spatial", errs)
    regs = s.get("region_scale_factors")
    if not isinstance(regs, list) or not regs:
        errs.append("spatial.region_scale_factors must be a non-empty list")
        return
    names = set()
    for i, r in enumerate(regs):
        w = f"spatial.region_scale_factors[{i}]"
        if not isinstance(r, dict):
            errs.append(f"{w} must be an object")
            continue
        _unknown(r, {"name", "scale", "bbox_xy_m", "assumed"}, w, errs)
        if not isinstance(r.get("name"), str) or not r["name"].strip():
            errs.append(f"{w}.name must be a non-empty string")
        elif r["name"] in names:
            errs.append(f"{w}.name {r['name']!r} is duplicated")
        else:
            names.add(r["name"])
        if not (_is_num(r.get("scale")) and r["scale"] > 0):
            errs.append(f"{w}.scale must be a finite number > 0 (dimensionless multiplier)")
        if not isinstance(r.get("assumed"), bool):
            errs.append(f"{w}.assumed is required and must be a boolean")
        if "bbox_xy_m" in r:
            b = r["bbox_xy_m"]
            if not (isinstance(b, list) and len(b) == 4 and all(_is_num(x) for x in b)
                    and b[0] < b[2] and b[1] < b[3]):
                errs.append(f"{w}.bbox_xy_m must be [xmin, ymin, xmax, ymax] in metres, "
                            "show frame, with xmin < xmax and ymin < ymax")


def _check_temporal(t, errs: list[str]) -> None:
    if not isinstance(t, dict):
        errs.append("temporal must be an object")
        return
    _unknown(t, {"window"}, "temporal", errs)
    w = t.get("window")
    if not isinstance(w, dict):
        errs.append("temporal.window must be an object")
        return
    _unknown(w, {"start_s", "end_s", "assumed"}, "temporal.window", errs)
    if not (_is_num(w.get("start_s")) and w["start_s"] >= 0):
        errs.append("temporal.window.start_s must be a finite number >= 0 (seconds of show time)")
    if not (_is_num(w.get("end_s")) and _is_num(w.get("start_s")) and w["end_s"] > w["start_s"]):
        errs.append("temporal.window.end_s must be a finite number greater than start_s")
    if not isinstance(w.get("assumed"), bool):
        errs.append("temporal.window.assumed is required and must be a boolean")


def validation_errors(raw) -> list[str]:
    """All problems found in a parsed file; empty if valid. Never raises on malformed input."""
    if not isinstance(raw, dict):
        return ["file must contain a JSON object"]
    errs: list[str] = []
    _unknown(raw, TOP_KEYS, "file", errs)
    for k in TOP_REQUIRED:
        if k not in raw:
            errs.append(f"missing key {k!r}" + (" (provenance is mandatory)"
                                                if k == "provenance" else ""))
    sv = raw.get("schema_version")
    if "schema_version" in raw and (isinstance(sv, bool) or sv != SCHEMA_VERSION):
        errs.append(f"unsupported schema_version {sv!r} (this code reads {SCHEMA_VERSION})")
    if "model" in raw and raw["model"] != MODEL_ID:
        errs.append(f"model must be {MODEL_ID!r}, got {raw['model']!r}")
    if "provenance" in raw:
        _check_provenance(raw["provenance"], errs)
    if "parameters" in raw:
        _check_parameters(raw["parameters"], errs)
    if "spatial" in raw:
        _check_spatial(raw["spatial"], errs)
    if "temporal" in raw:
        _check_temporal(raw["temporal"], errs)
    return errs


def from_dict(raw) -> GnssParams:
    errs = validation_errors(raw)
    if errs:
        raise GnssParamsError("invalid GNSS parameter file:\n" + "\n".join(errs))
    par = raw["parameters"]
    return GnssParams(
        provenance=dict(raw["provenance"]),
        values={n: float(par[n]["value"]) for n in PARAMS},
        assumed={n: par[n]["assumed"] for n in PARAMS},
        spatial=json.loads(json.dumps(raw["spatial"])) if "spatial" in raw else None,
        temporal=json.loads(json.dumps(raw["temporal"])) if "temporal" in raw else None,
    )


def _no_duplicates(pairs):
    d: dict = {}
    for k, v in pairs:
        if k in d:
            raise GnssParamsError(f"duplicate key {k!r} in JSON object")
        d[k] = v
    return d


def _no_constants(name):
    raise GnssParamsError(f"non-finite JSON constant {name} is not allowed")


def load(path: str | Path) -> GnssParams:
    text = read_text_limited(path, max_bytes=1024 * 1024)
    try:
        raw = json.loads(text, object_pairs_hook=_no_duplicates, parse_constant=_no_constants)
    except json.JSONDecodeError as exc:
        raise GnssParamsError(f"{path}: not valid JSON: {exc}") from exc
    return from_dict(raw)


def save(params: GnssParams, path: str | Path) -> None:
    errs = validation_errors(params.to_dict())
    if errs:
        raise GnssParamsError("refusing to save an invalid GNSS parameter file:\n" + "\n".join(errs))
    Path(path).write_text(json.dumps(params.to_dict(), indent=2) + "\n", encoding="utf-8")


def to_stress_config(params: GnssParams, base: StressConfig | None = None, *,
                     ignore_unsupported: bool = False) -> StressConfig:
    """Apply the seven GNSS fields to ``base`` (default ``StressConfig()``).

    A file carrying ``spatial`` or ``temporal`` blocks is refused unless ``ignore_unsupported``:
    ``StressConfig`` cannot express them, and silently dropping them would misreport the model.
    """
    unsupported = [n for n in ("spatial", "temporal") if getattr(params, n) is not None]
    if unsupported and not ignore_unsupported:
        raise GnssParamsError(
            f"file carries {', '.join(unsupported)} data that StressConfig cannot use today; "
            "pass ignore_unsupported=True to drop it knowingly")
    cfg = base if base is not None else StressConfig()
    return cfg.with_(**{PARAMS[n][0]: params.values[n] for n in PARAMS})


def from_stress_config(cfg: StressConfig, provenance: dict, assumed: bool | dict = True
                       ) -> GnssParams:
    """Capture the GNSS fields of ``cfg``. ``assumed`` must be stated (bool for all, or a dict)."""
    flags = ({n: assumed for n in PARAMS} if isinstance(assumed, bool) else dict(assumed))
    p = GnssParams(provenance=dict(provenance),
                   values={n: float(getattr(cfg, PARAMS[n][0])) for n in PARAMS},
                   assumed=flags)
    errs = validation_errors(p.to_dict()) if set(flags) == set(PARAMS) else [
        f"assumed must give a flag for every parameter: {sorted(PARAMS)}"]
    if errs:
        raise GnssParamsError("cannot build GNSS parameters:\n" + "\n".join(errs))
    return p
