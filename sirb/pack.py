"""Evidence pack: one folder with the plan, its checks, the diagnosis, a repair log and a one page
bilingual summary, plus a hash list and an optional detached signature.

Simulation and planning only. A pack is a record of what was computed under stated assumptions;
it is not an approval, a certificate of airworthiness or a statement about any real aircraft. A
signature proves which key signed the hash list, that is, the origin of the files; it does not
show that the show is safe. Signing needs the optional package ``cryptography`` (extra ``sign``);
the runtime dependencies of Sirb stay numpy and scipy.
"""

from __future__ import annotations

import hashlib
import html
import json
from pathlib import Path

from .disturbance import StressConfig
from .doctor import diagnose
from .envinfo import environment
from .export import build_export, save_export
from .gnss_band import band
from .planner.baseline import plan_baseline
from .repair import repair
from .verify2 import verify2

BANNER_EN = ("Simulation and planning only. Not flight certified. No claim of regulatory compliance. "
             "Real shows need an operator safety case and approvals from the relevant authorities.")
BANNER_AR = ("محاكاة وتخطيط فقط. غير معتمد للطيران. لا يُدّعى أي امتثال تنظيمي. تحتاج العروض الحقيقية "
             "إلى ملف سلامة من المشغّل وموافقات الجهات المختصة.")
LIMITATIONS = """Limitations of this evidence pack

1. Everything under stress, repair and the GNSS band is SIMULATED under assumed disturbance models.
2. The relative GNSS error between nearby receivers is not measured by this project; the values
   used are assumptions and every stressed verdict is conditional on them.
3. The exact and interval verifiers check separation between points on the planned polynomials; they
   do not model vehicle shape, downwash (unless the optional rule is run), radio, or control.
4. Tracking in the stress model is a generic second order loop; PX4 software in the loop results
   exist for 2 to 4 vehicles only and are not hardware evidence.
5. A hash list proves integrity of these files; a signature (if present) proves the origin of the
   hash list. Neither shows that a show is safe.
6. Not flight certified. No regulatory compliance claim. No partnership or approval is implied.
"""


def _sha(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def _e(x) -> str:
    return html.escape(str(x))


def summary_html(show: dict, ex: dict, v2, diag: dict, rep: dict | None, gband: dict | None,
                 target: float) -> str:
    st = diag["stress"]
    lo, hi = st["p_violation_ci95"]
    top = st["pair_ranking"][:3]
    pairs = ", ".join(f"{a['pair'][0]}-{a['pair'][1]}" for a in top) or "-"
    if rep is None:
        rep_en = rep_ar = "Not needed: the simulated upper bound is already within the target."
        rep_ar = "غير لازم: الحد الأعلى المحاكى ضمن الهدف أصلًا."
    else:
        f = rep["after"]
        rep_en = (f"{rep['n_moves']} move(s); P(violation) {rep['before']['p_violation']:.3f} to "
                  f"{f['p_violation']:.3f}; extra duration {rep['cost']['extra_duration_s']:+.1f} s, "
                  f"extra path {rep['cost']['extra_path_length_m']:+.0f} m; status {rep['status']}.")
        rep_ar = (f"{rep['n_moves']} تعديل؛ احتمال المخالفة من {rep['before']['p_violation']:.3f} إلى "
                  f"{f['p_violation']:.3f}؛ مدة إضافية {rep['cost']['extra_duration_s']:+.1f} ث، "
                  f"مسار إضافي {rep['cost']['extra_path_length_m']:+.0f} م.")
    dep = (gband or {}).get("depends_on_parameter")
    dep_en = ("The decision depends on the unmeasured relative GNSS error: some assumed values meet the "
              "target and some do not." if dep else
              "Within the assumed grid the decision does not change with the unmeasured relative GNSS error.")
    dep_ar = ("القرار يعتمد على خطأ GNSS النسبي غير المقاس: بعض القيم المفترضة تحقق الهدف وبعضها لا." if dep else
              "ضمن الشبكة المفترضة لا يتغير القرار بتغير خطأ GNSS النسبي غير المقاس.")
    rows_en = [("Exact verifier", f"{ex['verdict']}, minimum separation {ex['min_separation_m']:.3f} m "
                                  f"against {ex['threshold_m']} m"),
               ("Second verifier (interval)", f"{v2.verdict}, lower bound {v2.lower_bound:.3f} m"),
               ("Simulated P(violation)", f"{st['p_violation']:.3f} (Wilson 95%: {lo:.3f} to {hi:.3f}); "
                                          f"target upper bound {target}"),
               ("Pairs driving the risk", pairs), ("Repair", rep_en), ("Parameter dependence", dep_en)]
    rows_ar = [("المدقق الدقيق", f"{ex['verdict']}، أقل تباعد {ex['min_separation_m']:.3f} م مقابل {ex['threshold_m']} م"),
               ("المدقق الثاني (فترات)", f"{v2.verdict}، الحد الأدنى {v2.lower_bound:.3f} م"),
               ("احتمال المخالفة المحاكى", f"{st['p_violation']:.3f} (ويلسون 95٪: {lo:.3f} إلى {hi:.3f})؛ الحد الأعلى المستهدف {target}"),
               ("الأزواج الأكثر إسهامًا", pairs), ("الإصلاح", rep_ar), ("الاعتماد على المعامل", dep_ar)]

    def table(rows):
        return "".join(f"<tr><th>{_e(k)}</th><td>{_e(v)}</td></tr>" for k, v in rows)

    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Sirb evidence summary</title>
<style>
:root {{ --bg:#fff7f9; --card:#ffffff; --ink:#3b2a30; --muted:#7a5d66; --line:#f3d3dd; --accent:#d77a95; }}
@media (prefers-color-scheme: dark) {{ :root {{ --bg:#2a1d22; --card:#34242a; --ink:#f7e7ec; --muted:#d3b2bd; --line:#5a3d47; --accent:#f0a3b8; }} }}
body {{ margin:0; background:var(--bg); color:var(--ink); font:16px/1.55 system-ui, sans-serif; }}
main {{ max-width:880px; margin:0 auto; padding:24px 16px; }}
section {{ background:var(--card); border:1px solid var(--line); border-radius:10px; padding:16px 20px; margin:16px 0; }}
h1 {{ font-size:1.3rem; color:var(--accent); margin:0 0 4px; }} h2 {{ font-size:1.05rem; margin:0 0 8px; }}
table {{ border-collapse:collapse; width:100%; }} th, td {{ text-align:start; padding:6px 8px; border-top:1px solid var(--line); vertical-align:top; }}
th {{ width:34%; color:var(--muted); font-weight:600; }} .banner {{ color:var(--muted); font-size:.92rem; }}
</style></head><body><main>
<h1>{_e(show['name'])}: evidence summary</h1>
<p class="banner">{_e(BANNER_EN)}</p>
<section lang="en"><h2>Summary</h2><table>{table(rows_en)}</table>
<p class="banner">Fleet hash {_e(diag['fleet_sha256'])}. Stress results are simulated under assumed models.</p></section>
<section lang="ar" dir="rtl"><h2>الملخص (مسودة ترجمة للمراجعة)</h2><table>{table(rows_ar)}</table>
<p class="banner">{_e(BANNER_AR)}</p></section>
<section lang="en"><h2>What this page does not say</h2>
<p>It does not say the show is safe. A hash list proves integrity and a signature proves origin; neither proves safety.</p></section>
</main></body></html>
"""


def build_pack(show: dict, out_dir: str | Path, cfg: StressConfig, *, trials: int = 100,
               target: float = 0.05, do_repair: bool = True, with_band: bool = True,
               seed: int = 20261016, repair_budget_s: float = 300.0) -> dict:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    fleet, _ = plan_baseline(show)
    (out / "show.json").write_text(json.dumps(show, indent=1), encoding="utf-8")
    bundle = build_export(show, fleet)
    save_export(out / "export.json", bundle)
    d_min = show["limits"]["min_separation_m"]
    v2 = verify2(fleet, d_min)
    (out / "verification.json").write_text(json.dumps({
        "exact_certificate": bundle["certificate"],
        "interval_verifier": {"verdict": v2.verdict, "lower_bound_m": v2.lower_bound,
                              "upper_bound_m": v2.upper_bound, "tolerance": v2.tol}}, indent=1, default=float))
    diag = diagnose(show, fleet, cfg, trials=trials, seed=seed, target=target)
    (out / "diagnosis.json").write_text(json.dumps(diag, indent=1, default=float))
    rep = None
    if do_repair and diag["stress"]["exceeds_target"]:
        rep, show2, fleet2 = repair(show, cfg, target=target, time_budget_s=repair_budget_s)
        rep["environment"] = environment({"search": rep["search"]})
        (out / "repair_log.json").write_text(json.dumps(rep, indent=1, default=float))
        save_export(out / "export_repaired.json", build_export(show2, fleet2))
    else:
        (out / "repair_log.json").write_text(json.dumps({"status": "not_run" if not do_repair else
                                                         "not_needed"}, indent=1))
    gb = band(fleet, d_min, cfg, trials=min(trials, 100), seed=seed, target=target) if with_band else None
    (out / "stress_summary.json").write_text(json.dumps({
        "label": "SIMULATED under assumed disturbance models; not flight evidence",
        "stress": diag["stress"], "gnss_band": gb, "environment": diag["environment"]}, indent=1, default=float))
    (out / "limitations.txt").write_text(LIMITATIONS, encoding="utf-8")
    (out / "summary.html").write_text(summary_html(show, diag["exact"], v2, diag, rep, gb, target),
                                      encoding="utf-8")
    return write_hashes(out)


def write_hashes(out: Path) -> dict:
    files = sorted(p for p in out.iterdir() if p.is_file() and p.name not in ("SHA256SUMS", "SHA256SUMS.sig"))
    lines = [f"{_sha(p)}  {p.name}" for p in files]
    (out / "SHA256SUMS").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {"files": [p.name for p in files], "hash_list": str(out / "SHA256SUMS")}


def check_hashes(out: str | Path) -> list[str]:
    out = Path(out)
    bad = []
    for line in (out / "SHA256SUMS").read_text(encoding="utf-8").splitlines():
        h, name = line.split("  ", 1)
        p = out / name
        if not p.is_file() or _sha(p) != h:
            bad.append(name)
    return bad


def keygen(path: str | Path) -> str:
    from cryptography.hazmat.primitives import serialization
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
    k = Ed25519PrivateKey.generate()
    Path(path).write_bytes(k.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                           serialization.NoEncryption()))
    pub = k.public_key().public_bytes(serialization.Encoding.Raw, serialization.PublicFormat.Raw)
    Path(str(path) + ".pub").write_text(pub.hex() + "\n")
    return pub.hex()


def sign(out: str | Path, key_path: str | Path) -> str:
    from cryptography.hazmat.primitives import serialization
    key = serialization.load_pem_private_key(Path(key_path).read_bytes(), password=None)
    sig = key.sign((Path(out) / "SHA256SUMS").read_bytes())
    (Path(out) / "SHA256SUMS.sig").write_text(sig.hex() + "\n")
    return sig.hex()


def verify_signature(out: str | Path, pub_hex: str) -> bool:
    from cryptography.exceptions import InvalidSignature
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey
    pub = Ed25519PublicKey.from_public_bytes(bytes.fromhex(pub_hex.strip()))
    sig = bytes.fromhex((Path(out) / "SHA256SUMS.sig").read_text().strip())
    try:
        pub.verify(sig, (Path(out) / "SHA256SUMS").read_bytes())
        return True
    except InvalidSignature:
        return False
