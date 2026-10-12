import copy
import json
from dataclasses import fields
from pathlib import Path

import pytest

from sirb import gnss_params as G
from sirb.disturbance import StressConfig

ROOT = Path(__file__).resolve().parent.parent
DEFAULT = ROOT / "shows" / "gnss_assumed_default.json"
SCHEMA = ROOT / "sirb" / "schemas" / "gnss_params.schema.json"
GNSS_FIELDS = [f[0] for f in G.PARAMS.values()]


def raw():
    return json.loads(DEFAULT.read_text(encoding="utf-8"))


def test_default_file_reproduces_todays_defaults():
    p = G.load(DEFAULT)
    cfg = G.to_stress_config(p)
    ref = StressConfig()
    for f in GNSS_FIELDS:
        assert getattr(cfg, f) == getattr(ref, f), f
    assert cfg == ref  # non-GNSS fields untouched
    assert all(p.assumed.values()) and len(p.assumed) == 7
    assert "docs/METHODS.md" in p.provenance["url"]


def test_gnss_fields_in_map_are_exactly_the_gnss_fields_of_stressconfig():
    names = {f.name for f in fields(StressConfig) if f.name.startswith("gnss_")}
    assert set(GNSS_FIELDS) == names


def test_roundtrip_via_stressconfig_and_file(tmp_path):
    cfg = StressConfig(gnss_white_sigma=0.02, gnss_deg_fraction=0.25, gnss_bias_sigma=0.5,
                       gnss_bias_tau=40.0, gnss_loss_fraction=0.1, gnss_loss_s=20.0,
                       gnss_dead_reckoning_rate=0.5)
    p = G.from_stress_config(cfg, raw()["provenance"], assumed=True)
    out = tmp_path / "p.json"
    G.save(p, out)
    back = G.to_stress_config(G.load(out))
    for f in GNSS_FIELDS:
        assert getattr(back, f) == getattr(cfg, f)


def test_base_config_preserved():
    base = StressConfig(w20=6.0, motor_failures=2)
    cfg = G.to_stress_config(G.load(DEFAULT), base)
    assert cfg.w20 == 6.0 and cfg.motor_failures == 2


def test_example_validates_against_json_schema():
    jsonschema = pytest.importorskip("jsonschema")
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.validate(raw(), schema)


def mutate(fn):
    d = raw()
    fn(d)
    return d


BAD = {
    "no provenance": lambda d: d.pop("provenance"),
    "provenance missing url": lambda d: d["provenance"].pop("url"),
    "provenance missing licence": lambda d: d["provenance"].pop("licence"),
    "provenance missing derived_by": lambda d: d["provenance"].pop("derived_by"),
    "empty source": lambda d: d["provenance"].update(source_name="  "),
    "bad date": lambda d: d["provenance"].update(retrieval_date="2026-13-40"),
    "date format": lambda d: d["provenance"].update(retrieval_date="12/10/2026"),
    "unknown provenance key": lambda d: d["provenance"].update(extra=1),
    "assumed missing": lambda d: d["parameters"]["bias_tau"].pop("assumed"),
    "assumed not bool": lambda d: d["parameters"]["bias_tau"].update(assumed="true"),
    "wrong unit": lambda d: d["parameters"]["white_sigma"].update(unit="cm"),
    "unit missing": lambda d: d["parameters"]["white_sigma"].pop("unit"),
    "value string": lambda d: d["parameters"]["white_sigma"].update(value="0.015"),
    "value bool": lambda d: d["parameters"]["white_sigma"].update(value=True),
    "negative sigma": lambda d: d["parameters"]["white_sigma"].update(value=-0.1),
    "fraction above one": lambda d: d["parameters"]["degraded_fraction"].update(value=1.5),
    "tau zero": lambda d: d["parameters"]["bias_tau"].update(value=0.0),
    "duration zero": lambda d: d["parameters"]["dropout_duration"].update(value=0),
    "unknown parameter": lambda d: d["parameters"].update(mystery={"value": 1, "unit": "m",
                                                                    "assumed": True}),
    "missing parameter": lambda d: d["parameters"].pop("dead_reckoning_rate"),
    "unknown param key": lambda d: d["parameters"]["bias_tau"].update(comment="x"),
    "unknown top key": lambda d: d.update(extra=1),
    "future schema": lambda d: d.update(schema_version=2),
    "bool schema": lambda d: d.update(schema_version=True),
    "wrong model": lambda d: d.update(model="other"),
    "spatial bad scale": lambda d: d.update(spatial={"region_scale_factors": [
        {"name": "a", "scale": 0, "assumed": True}]}),
    "spatial no assumed": lambda d: d.update(spatial={"region_scale_factors": [
        {"name": "a", "scale": 2.0}]}),
    "spatial dup": lambda d: d.update(spatial={"region_scale_factors": [
        {"name": "a", "scale": 2.0, "assumed": True}, {"name": "a", "scale": 1.0,
                                                       "assumed": True}]}),
    "spatial bad bbox": lambda d: d.update(spatial={"region_scale_factors": [
        {"name": "a", "scale": 2.0, "assumed": True, "bbox_xy_m": [10, 0, 0, 5]}]}),
    "temporal reversed": lambda d: d.update(temporal={"window": {
        "start_s": 20, "end_s": 10, "assumed": True}}),
    "temporal no assumed": lambda d: d.update(temporal={"window": {"start_s": 1, "end_s": 2}}),
}


@pytest.mark.parametrize("name", sorted(BAD))
def test_invalid_files_rejected(name):
    d = mutate(BAD[name])
    assert G.validation_errors(d), name
    with pytest.raises(G.GnssParamsError):
        G.from_dict(d)


def test_non_object_and_garbage(tmp_path):
    for x in (None, [], "s", 3):
        assert G.validation_errors(x)
    f = tmp_path / "a.json"
    f.write_text("{not json", encoding="utf-8")
    with pytest.raises(G.GnssParamsError):
        G.load(f)


def test_duplicate_keys_and_nan_rejected(tmp_path):
    f = tmp_path / "a.json"
    f.write_text('{"schema_version": 1, "schema_version": 1}', encoding="utf-8")
    with pytest.raises(G.GnssParamsError, match="duplicate"):
        G.load(f)
    text = DEFAULT.read_text(encoding="utf-8").replace('"value": 0.015', '"value": NaN')
    f.write_text(text, encoding="utf-8")
    with pytest.raises(G.GnssParamsError, match="non-finite"):
        G.load(f)


def test_optional_blocks_validate_but_refuse_conversion_by_default():
    d = raw()
    d["spatial"] = {"region_scale_factors": [
        {"name": "north", "scale": 2.0, "assumed": True, "bbox_xy_m": [-50, 0, 50, 100]}]}
    d["temporal"] = {"window": {"start_s": 10.0, "end_s": 40.0, "assumed": True}}
    p = G.from_dict(d)
    assert p.to_dict()["spatial"] == d["spatial"]
    with pytest.raises(G.GnssParamsError, match="cannot use"):
        G.to_stress_config(p)
    assert G.to_stress_config(p, ignore_unsupported=True) == StressConfig()


def test_errors_are_reported_together():
    d = raw()
    d.pop("provenance")
    d["parameters"]["white_sigma"]["unit"] = "cm"
    d["extra"] = 1
    with pytest.raises(G.GnssParamsError) as ei:
        G.from_dict(d)
    msg = str(ei.value)
    assert "provenance" in msg and "unit" in msg and "unknown key" in msg


def test_from_stress_config_requires_complete_assumed_flags():
    prov = raw()["provenance"]
    with pytest.raises(G.GnssParamsError):
        G.from_stress_config(StressConfig(), prov, assumed={"white_sigma": True})
    with pytest.raises(G.GnssParamsError):
        G.from_stress_config(StressConfig(gnss_deg_fraction=2.0), prov, assumed=True)
    with pytest.raises(G.GnssParamsError):
        G.from_stress_config(StressConfig(), {"source_name": "x"}, assumed=True)


def test_save_refuses_invalid(tmp_path):
    p = G.load(DEFAULT)
    p.values["bias_tau"] = -1.0
    with pytest.raises(G.GnssParamsError):
        G.save(p, tmp_path / "x.json")
    assert not (tmp_path / "x.json").exists()


def test_schema_and_code_agree_on_parameter_names():
    schema = json.loads(SCHEMA.read_text(encoding="utf-8"))
    assert set(schema["properties"]["parameters"]["properties"]) == set(G.PARAMS)
    assert copy.deepcopy(schema["properties"]["schema_version"]["const"]) == G.SCHEMA_VERSION
