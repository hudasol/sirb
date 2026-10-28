"""Generate viewer/results.html from results/*.json and viewer/data/index.json.

Every number on the page is read from a result file; each row names its source file, the first 12
characters of that file's SHA-256 (line endings normalised) and the seed. Nothing is typed by hand
except the wording of claims and the limitations. Static page, no inline data, same style and
language switch as methods.html.     python scripts/make_results_page.py
"""

from __future__ import annotations

import base64
import hashlib
import html
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
R = ROOT / "results"
REPO = "https://github.com/hudasol/sirb/blob/main"


def load(name):
    raw = (R / name).read_bytes().replace(b"\r\n", b"\n")
    return json.loads(raw), hashlib.sha256(raw).hexdigest()[:12]


def esc(s):
    return html.escape(str(s), quote=True)


def src(name, sha, doc):
    return (f'<a href="{REPO}/results/{esc(name)}"><code>{esc(name)}</code></a> '
            f'<span class="meta">sha256 {sha}</span> '
            f'<a href="{REPO}/docs/{esc(doc)}">{esc(doc)}</a>')


def ci(lo, hi):
    return f"{lo:.2f} to {hi:.2f}"


def bil(en, ar):
    return f'<span class="en">{en}</span><span class="ar">{ar}</span>'


ROWS: list[tuple[str, str, str, str, str]] = []      # id, claim html, value html, source html, seed


def row(rid, en, ar, value, source, seed):
    ROWS.append((rid, bil(en, ar), value, source, seed))


# ---- verification evidence
bm, bsha = load("benchmark_v2.json")
S = bm["strata"]
tot = sum(s["n"] for s in S.values())
ex = sum(s["exact_wrong"]["count"] for s in S.values())
iw = sum(s["interval_wrong"]["count"] for s in S.values())
iu = sum(s["interval_undecided"]["count"] for s in S.values())
bsrc = src("benchmark_v2.json", bsha, "BENCHMARK_V2.md")
row("exact-wrong", "Exact verifier: wrong verdicts on constructed cases with known truth",
    "المدقِّق الدقيق: أحكام خاطئة على حالات مبنية بحقيقة معروفة",
    f"{ex} of {tot}", bsrc, bm["seed"])
row("interval-wrong", "Independent interval verifier: wrong verdicts, and cases left undecided",
    "المدقِّق الفاصلي المستقل: أحكام خاطئة وحالات بلا قرار",
    f"{iw} wrong, {iu} undecided of {tot}", bsrc, bm["seed"])
rz = S["razor_violating"]
row("sampled-razor", "Sampled check at a 0.05 s step missing violations of depth 1e-6 to 1e-4 m",
    "فحص بالعيّنات بخطوة 0.05 ث يفوّت انتهاكات بعمق 1e-6 إلى 1e-4 م",
    f"{rz['fixed_wrong']['0.05']['count']} of {rz['n']} missed", bsrc, bm["seed"])
row("sampled-matched", "Sampled check given the same compute time as the exact verifier, same cases",
    "فحص بالعيّنات بنفس زمن الحساب للمدقِّق الدقيق، الحالات نفسها",
    f"{rz['matched_wrong']['count']} of {rz['n']} missed", bsrc, bm["seed"])
bl, lsha = load("baselines.json")
fp = bl["sampled_false_pass"]
row("sampled-crossings", "Sampled check at a 0.5 s step passing fleets that fail the exact check",
    "فحص بالعيّنات بخطوة 0.5 ث يُجيز أساطيل تفشل في الفحص الدقيق",
    f"{fp['false_pass']['0.5']} of {fp['trials']}", src("baselines.json", lsha, "BASELINES.md"), "see file")

# ---- stress evidence
gc, gsha = load("gnss_common.json")
gsrc = src("gnss_common.json", gsha, "GNSS_COMMON.md")
rows_g = {r["common_fraction"]: r for r in gc["rows"]}
for f in (0.0, 0.95, 1.0):
    r = rows_g[f]
    row(f"gnss-common-{f}", f"P(violation), GNSS error common fraction {f} of the bias variance, {gc['trials']} trials",
        f"احتمال الانتهاك، نسبة مشتركة {f} من تباين انحياز GNSS، {gc['trials']} تجربة",
        f"{r['p_violation']:.2f} ({ci(*r['p_violation_ci95'])})", gsrc, gc["seed"])
ss, ssha = load("stress_search.json")
for key, en, ar in (("random", "random search", "بحث عشوائي"), ("evolution", "evolution strategy", "استراتيجية تطور")):
    e = ss[key]["evaluation"]
    row(f"search-{key}", f"Worst case search ({en}) inside the declared parameter box: P(violation) on separate seeds",
        f"البحث عن أسوأ حالة ({ar}) داخل صندوق المعاملات المعلن: احتمال الانتهاك على بذور منفصلة",
        f"{e['p_violation']:.2f} ({ci(*e['p_violation_ci95'])})",
        src("stress_search.json", ssha, "STRESS_CREDIBILITY.md"), ss["seed_eval"])
fa, fsha = load("stress_faults.json")
fsrc = src("stress_faults.json", fsha, "STRESS_CREDIBILITY.md")
want = {"comms_loss 5%, hold, delay 0 s": ("Comms loss in 5% of drones, hold behaviour, no detection delay",
                                           "فقدان الاتصال في 5% من الطائرات، سلوك التثبيت، بلا تأخر كشف"),
        "comms_loss 5%, corridor, delay 0 s": ("Comms loss in 5% of drones, corridor behaviour, no detection delay",
                                               "فقدان الاتصال في 5% من الطائرات، سلوك الممر، بلا تأخر كشف"),
        "combined faults, delay 0 s": ("Combined faults (comms, battery, one motor, GNSS dropout, GNSS bias)",
                                       "أعطال مجتمعة (اتصال وبطارية ومحرك واحد وانقطاع GNSS وانحياز GNSS)")}
for r in fa["rows"]:
    if r["case"] in want:
        en, ar = want[r["case"]]
        row("fault-" + r["case"].split(",")[0].split()[0] + "-" + r["case"].split(",")[1].strip().split()[0],
            f"{en}: P(violation)", f"{ar}: احتمال الانتهاك",
            f"{r['p_violation']:.2f} ({ci(*r['p_violation_ci95'])})", fsrc, fa["seed"])

# ---- model fidelity and autopilot replay
fi, fisha = load("fidelity.json")
fsrc2 = src("fidelity.json", fisha, "MODEL_FIDELITY.md")
row("delay-margin", "Delay margin of the assumed tracking loop (feedback delay above this makes it unstable)",
    "هامش التأخر لحلقة التتبّع المفترضة (تأخر أعلى منه يجعلها غير مستقرة)",
    f"{fi['delay_margin']['delay_margin_s']:.3f} s", fsrc2, "analytic")
w = fi["required_feedback_accel"]["wind 9 m/s"]
row("wind-authority", "Wind 9 m/s: median per drone peak feedback acceleration the loop asks for",
    "رياح 9 م/ث: الوسيط لذروة تسارع التغذية الراجعة المطلوبة لكل طائرة",
    f"{w['drone_peaks_p50']:.2f} m/s^2", fsrc2, fi["seed"])
sd, ssha2 = load("sitl_replay_4.json")
row("sitl-4", "PX4 simulation in hardware, 4 vehicles, default parameters: planned versus flown minimum separation",
    "PX4 بمحاكاة داخل العتاد، 4 مركبات، معاملات افتراضية: أقل تباعد مخطط مقابل المُحلَّق",
    f"{sd['planned_min_separation_exact_m']:.2f} m planned, {sd['flown_min_separation_m']:.2f} m flown",
    src("sitl_replay_4.json", ssha2, "SITL.md"), "one real time run")

# ---- events from the viewer scenarios
idx = json.loads((ROOT / "viewer" / "data" / "index.json").read_text(encoding="utf-8"))
ev_rows = []
for show in idx["shows"]:
    for sc in show["scenarios"]:
        for e in sc.get("events", []):
            ev_rows.append((show["id"], sc["id"], e["drone"], e["kind"], e["t"]))

LIMITS = [
    ("A pass on this page means the nominal trajectory passed the specified separation check for the planned "
     "polynomial paths. It is not a statement that a fleet is safe to fly and not an approval.",
     "النجاح في هذه الصفحة يعني أن المسار الاسمي اجتاز فحص التباعد المحدد للمسارات متعددة الحدود المخططة. "
     "وهو ليس إقرارًا بأن الأسطول آمن للطيران وليس موافقة."),
    ("Stressed results come from assumed disturbance models. The GNSS error correlation between drones is "
     "not sourced and decides the verdict (P(violation) 1.00 up to a common fraction of 0.95, 0.00 at 1.00).",
     "نتائج الإجهاد مبنية على نماذج اضطراب مفترضة. ارتباط خطأ GNSS بين الطائرات غير موثّق بمصدر وهو يحسم النتيجة."),
    ("The tracking loop is a second order model. Latency and an acceleration cap are opt in and were studied on "
     "three configurations only. The wind verdict depends on the vehicle having enough feedback authority, and "
     "the loop is unstable beyond its delay margin; neither was measured on a vehicle.",
     "حلقة التتبّع نموذج من الدرجة الثانية. التأخر وحدّ التسارع اختياريان وقد دُرسا في ثلاث حالات فقط. "
     "نتيجة الرياح تفترض قدرة تحكم كافية في المركبة، وتصبح الحلقة غير مستقرة فوق هامش التأخر، ولم يُقَس أي منهما على مركبة."),
    ("Wind drag uses the wind velocity, not the velocity relative to the air, so drift sizes when authority "
     "runs out are not meaningful.",
     "سحب الرياح يستخدم سرعة الرياح لا السرعة بالنسبة للهواء، لذا لا يُعتدّ بحجم الانجراف عند نفاد قدرة التحكم."),
    ("The PX4 replay is simulation only, one real time run per vehicle count with default parameters, and "
     "aligns logs by the wall clock. It is not evidence about any real vehicle.",
     "إعادة التشغيل على PX4 محاكاة فقط، بتشغيل واحد لكل عدد مركبات ومعاملات افتراضية وبمحاذاة السجلات بالساعة الفعلية. "
     "وليست دليلًا على أي مركبة حقيقية."),
    ("Monte Carlo rates come with Wilson 95% intervals; each scenario in the viewer is one fixed seed realisation.",
     "نسب مونت كارلو مقرونة بفواصل ويلسون 95%؛ وكل سيناريو في العارض هو تحقّق واحد بذرة ثابتة."),
    ("No real flight data is used anywhere. No regulatory property is claimed. A real show needs an operator "
     "safety case and approvals from the relevant UAE aviation authorities.",
     "لا تُستخدم أي بيانات طيران حقيقية. ولا يُدَّعى أي امتثال تنظيمي. تحتاج العروض الحقيقية إلى ملف سلامة "
     "تشغيلي وموافقات الجهات المختصة بالطيران في الإمارات."),
]

SCRIPT = """
import { STRINGS, getLang, storeLang, applyDocLang, makeT, translateDom } from "./i18n.js";
const q = new URLSearchParams(location.search).get("lang");
let lang = q === "ar" || q === "en" ? q : getLang();
let theme = "dark";
try { const v = localStorage.getItem("sirb.viewer.theme"); if (v === "dark" || v === "light") theme = v; else if (matchMedia("(prefers-color-scheme: light)").matches) theme = "light"; } catch (e) { if (matchMedia("(prefers-color-scheme: light)").matches) theme = "light"; }
function apply() {
  const t = makeT(lang);
  applyDocLang(lang); translateDom(document, t);
  document.documentElement.dataset.theme = theme;
  document.title = lang === "ar" ? "سرب: النتائج" : "Sirb results";
  const b = document.getElementById("langBtn"); b.textContent = t("lang_switch"); b.title = t("lang_switch_label"); b.lang = lang === "ar" ? "en" : "ar";
  document.getElementById("themeBtn").textContent = theme === "dark" ? t("theme_light") : t("theme_dark");
}
document.getElementById("langBtn").addEventListener("click", () => { lang = lang === "ar" ? "en" : "ar"; storeLang(lang); apply(); });
document.getElementById("themeBtn").addEventListener("click", () => { theme = theme === "dark" ? "light" : "dark"; try { localStorage.setItem("sirb.viewer.theme", theme); } catch (e) {} apply(); });
apply();
"""


def page() -> str:
    sha = "'sha256-" + base64.b64encode(hashlib.sha256(SCRIPT.encode("utf-8")).digest()).decode() + "'"
    p = [f'''<!doctype html>
<html lang="en" dir="ltr" data-theme="dark">
<head>
<meta charset="utf-8">
<meta http-equiv="Content-Security-Policy" content="default-src 'self'; script-src 'self' {sha}; style-src 'self' 'unsafe-inline'; img-src 'self' data:; connect-src 'self'; object-src 'none'; base-uri 'none'; form-action 'none'">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Sirb results</title>
<meta name="color-scheme" content="dark light">
<link rel="icon" href="data:,">
<link rel="stylesheet" href="style.css">
</head>
<body>
<div class="doc">
  <header class="nav">
    <a class="btn" id="backLink" href="index.html" data-i18n="m_back">Back to viewer</a>
    <button id="langBtn" type="button" class="btn"></button>
    <button id="themeBtn" type="button" class="btn" data-i18n-aria="theme_switch_label"></button>
  </header>
  <aside class="banner" id="safety-statement" aria-label="Simulation only">
    <strong class="en">Simulation only</strong><strong class="ar">محاكاة فقط</strong>
    <p class="en">Simulation and planning tool only. Not flight-certified; no claim of regulatory compliance. Real shows need an operator safety case and approvals from the relevant UAE aviation authorities. Simulation-validated only.</p>
    <p class="ar">أداة محاكاة وتخطيط فقط. غير معتمدة للطيران ولا تدّعي أي امتثال تنظيمي. تحتاج العروض الحقيقية إلى ملف سلامة تشغيلي وموافقات الجهات المختصة بالطيران في الإمارات. مُحقَّق منها بالمحاكاة فقط.</p>
  </aside>
  <main>
  <h1>{bil("Results and where they come from", "النتائج ومصدر كل منها")}</h1>
  <p class="note" data-i18n="m_lang_note"></p>
  <p class="en">Generated by <code>scripts/make_results_page.py</code> from the files in <code>results/</code>. Each row names its source file, the first 12 characters of the file's SHA-256 and the seed.</p>
  <p class="ar">تُولَّد الصفحة بالسكربت scripts/make_results_page.py من ملفات المجلد results/. يذكر كل صف ملف المصدر وأول 12 خانة من بصمة SHA-256 للملف والبذرة.</p>
  <section id="results">
    <h2>{bil("Results", "النتائج")}</h2>
    <div class="tbl-wrap"><table class="tbl">
      <thead><tr><th>{bil("Result", "النتيجة")}</th><th>{bil("Value", "القيمة")}</th><th>{bil("Source", "المصدر")}</th><th>{bil("Seed", "البذرة")}</th></tr></thead>
      <tbody>''']
    for rid, claim, value, source, seed in ROWS:
        p.append(f'<tr id="r-{esc(rid)}"><td>{claim}</td><td dir="ltr">{esc(value)}</td><td dir="ltr">{source}</td><td dir="ltr">{esc(seed)}</td></tr>')
    p.append(f'''      </tbody>
    </table></div>
  </section>
  <section id="events">
    <h2>{bil("Events in the viewer's stressed scenarios", "أحداث سيناريوهات الإجهاد في العارض")}</h2>
    <p class="en">Read from <code>viewer/data/index.json</code>. Times are seconds from the start of the show; drone numbers are those shown in the viewer.</p>
    <p class="ar">مقروءة من viewer/data/index.json. الأزمنة بالثواني من بداية العرض وأرقام الطائرات هي المعروضة في العارض.</p>
    <div class="tbl-wrap"><table class="tbl">
      <thead><tr><th>{bil("Show", "العرض")}</th><th>{bil("Scenario", "السيناريو")}</th><th>{bil("Drone", "الطائرة")}</th><th>{bil("Event", "الحدث")}</th><th>{bil("Time (s)", "الزمن (ث)")}</th></tr></thead>
      <tbody>''')
    for a, b, c, d, e in ev_rows:
        p.append(f'<tr><td dir="ltr">{esc(a)}</td><td dir="ltr">{esc(b)}</td><td dir="ltr">{esc(c)}</td><td dir="ltr">{esc(d)}</td><td dir="ltr">{e:.1f}</td></tr>')
    p.append(f'''      </tbody>
    </table></div>
  </section>
  <section id="limitations">
    <h2>{bil("Limitations", "القيود")}</h2>
    <ul>''')
    for en, ar in LIMITS:
        p.append(f"<li>{bil(esc(en), esc(ar))}</li>")
    p.append(f'''    </ul>
  </section>
  </main>
</div>
<script type="module">{SCRIPT}</script>
</body>
</html>
''')
    return "\n".join(p)


if __name__ == "__main__":
    (ROOT / "viewer" / "results.html").write_text(page(), encoding="utf-8")
    print("rows:", len(ROWS), "events:", len(ev_rows))
