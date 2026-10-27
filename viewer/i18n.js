// Bilingual strings (EN / AR) and small helpers shared by index.html and methods.html.
// Technical identifiers (d_min, SHA-256, JSON keys, drone ids) stay in Latin script.

export const STRINGS = {
  en: {
    app_title: "Sirb viewer",
    tagline: "Simulation-first drone-show planner, a separation verifier that shares no code with the planners, and a stress tester",
    skip: "Skip to the viewer",
    banner_title: "Simulation only",
    banner_en_original: "Original English statement (from index.json):",
    safety_statement:
      "Simulation and planning tool only. Not flight-certified; no claim of regulatory compliance. Real shows need an operator safety case and approvals from the relevant UAE aviation authorities. Simulation-validated only.",
    show: "Show",
    show_meta: "{n} drones, {dur} s, d_min = {dmin} m",
    language: "Language",
    lang_switch: "العربية",
    lang_switch_label: "Switch language to Arabic",
    theme: "Theme",
    theme_dark: "Dark",
    theme_light: "Light",
    theme_switch_label: "Toggle dark or light theme",
    methods: "Methods",
    methods_title: "How every number is computed",
    loading: "Loading data…",
    loading_show: "Loading {name}…",
    load_error: "Could not load viewer data: {msg}",
    load_hint:
      "The viewer fetches binary files, so it must be served over HTTP. From the repo root run: python -m http.server -d viewer 8000 and open http://localhost:8000/ (opening index.html directly via file:// does not work).",
    webgl_error: "WebGL is not available in this browser, so the 3D views cannot be drawn.",
    retry: "Retry",

    view_nominal: "Nominal plan",
    view_nominal_badge: "Exact verifier",
    view_stress: "Stressed scenario",
    view_stress_badge: "Simulation, ASSUMED models",
    scenario: "Scenario",
    canvas_label_a: "3D view of the nominal drone formation (plan checked against its declared limits). Drag to orbit, scroll to zoom.",
    canvas_label_b: "3D view of the stressed scenario, same camera as the nominal view. Drag to orbit, scroll to zoom.",
    readout_min_now: "Min nearest-neighbour distance now",
    readout_min_sofar: "Min nearest-neighbour distance so far",
    readout_below: "Drones below d_min now",
    readout_events: "Failure events triggered",
    unit_m: "m",
    of_n: "{k} of {n}",
    sampled_note: "Sampled every {dt} s from the viewer data; can miss minima between frames.",
    stress_label: "Simulation result under ASSUMED disturbance models. Not a guarantee.",
    nominal_label: "Nominal readouts are sampled frame values; the exact figure is in the verification record panel.",

    play: "Play",
    pause: "Pause",
    views_label: "3D views: nominal plan and stressed scenario",
    playback_label: "Playback controls",
    step_back: "Back one frame",
    step_fwd: "Forward one frame",
    restart: "Restart",
    speed: "Speed",
    timeline: "Timeline",
    time_readout: "Time {t} of {total} s",
    colour_mode: "Colour mode",
    mode_height: "By height",
    mode_sep: "By separation",
    height_legend: "Low",
    height_legend_hi: "High",
    legend_height: "Height: violet (ground) to pale cyan (highest)",
    legend_red: "Red: below d_min",
    legend_amber: "Amber: below 1.25 d_min",
    legend_green: "Green: at least 1.25 d_min (a display colour, not a safety rating)",
    legend_below_big: "Drones below d_min are also drawn larger",
    legend_geo: "Geofence outline (ground and ceiling)",
    legend_crowd: "Crowd line (solid)",
    legend_buffer: "Crowd buffer line, crowd.y + min distance (dashed)",
    legend_marker_motor: "Diamond: motor failure (ballistic fall)",
    legend_marker_comms: "Diamond: comms loss",
    legend_marker_other: "Diamond: other event",
    legend_size: "Drones are drawn enlarged for visibility (0.9 m radius); the modelled vehicle radius is 0.24 m.",
    keys_hint: "Timeline keys: arrows ±1 s, Shift+arrows ±1 frame, Home/End.",

    cert_title: "Verification record (not an approval or regulatory certificate)",
    cert_loading: "Loading verification record…",
    verdict: "Verdict",
    pass: "PASS",
    fail: "FAIL",
    verdict_note: "Means only that the paths as written meet the limits declared in the show file, several of which are assumptions. It is not an approval, a safety clearance or evidence that a real show would be safe.",
    cert_min_sep: "Min separation (exact)",
    cert_min_sep_at: "at t = {t} s, drones {a} and {b}",
    cert_dmin: "Declared threshold d_min",
    cert_dmin_assumed: "(an assumption)",
    cert_margin: "Margin over d_min",
    cert_recheck: "Recomputation check (same verifier code, re-run)",
    recheck_yes: "Passed: hash and numbers recomputed by the same code",
    recheck_no: "Not passed",
    cert_sha: "Fleet SHA-256",
    copy: "Copy",
    copied: "Copied",
    copy_failed: "Copy failed",
    show_full_hash: "Show full hash",
    cert_pairs: "Segments / pairs checked",
    cert_pairs_val: "{seg} segments, {pairs} drone pairs near enough to solve exactly",
    cert_dyn_title: "Dynamic limits (maximum reached / limit)",
    dyn_speed_h: "Horizontal speed (m/s)",
    dyn_speed_up: "Climb rate (m/s)",
    dyn_speed_down: "Descent rate (m/s)",
    dyn_accel: "Acceleration (m/s²)",
    dyn_jerk: "Jerk (m/s³)",
    dyn_geofence: "Geofence margin (m)",
    dyn_crowd: "Crowd margin (m)",
    cert_guarantees: "What the verifier guarantees (under its declared model)",
    cert_not: "What it does NOT guarantee",
    cert_statement: "Verification record statement",
    cert_text_note: "Text is quoted from certificate.json. \"Certificate\" is the file name; it is a verification record, not an approval.",
    cert_error: "Could not load certificate: {msg}",

    scen_title: "Scenario details",
    scen_assumed: "These are simulation results under ASSUMED models (wind, GNSS and failure parameters are assumptions, see Methods). They are not guarantees and not measurements.",
    scen_scope: "Scope: only distances between drones are evaluated here. Geofence containment, distance to the crowd, ground impact of falling drones, battery endurance, launch and landing are not evaluated under disturbance.",
    scen_min_nn: "Min nearest-neighbour distance over the run",
    scen_max_err: "Max tracking error",
    scen_seed: "Random seed (one fixed realisation)",
    scen_config: "Scenario parameters (assumed)",
    scen_events: "Events",
    scen_no_events: "No discrete failure events in this scenario.",
    scen_ballistic: "Ballistic (uncontrolled fall) drones",
    scen_none: "none",
    jump: "Jump to",
    event_row: "Drone {d}: {kind} at {t} s",
    kind_motor_failure: "Motor failure",
    kind_comms_loss: "Comms loss",
    kind_battery_low: "Low battery",
    kind_gnss_loss: "GNSS loss",
    cfg_w20: "Mean wind at 20 ft (m/s)",
    cfg_gnss_deg_fraction: "Share of drones with degraded GNSS",
    cfg_gnss_bias_sigma: "GNSS bias sigma (m)",
    cfg_motor_failures: "Motor failures (count)",
    cfg_comms_loss_fraction: "Comms-loss share of drones",
    cfg_behavior: "Failure behaviours",
    scen_nominal_note: "The nominal view shows the plan itself, as it passed the declared-limit check; nothing is perturbed.",

    foot_note: "Sirb: simulation-first. Not flight-certified. three.js (MIT) is vendored in vendor/.",
    // methods page
    m_back: "Back to viewer",
    m_title: "Methods: how every number in the viewer is computed",
    m_toc: "Contents",
    m_lang_note: "Arabic text gives the heading and a short summary of each section; the detailed English text is the authoritative version. Technical identifiers stay in Latin script.",
  },
  ar: {
    app_title: "عارض سرب",
    tagline: "مخطِّط عروض الطائرات المسيّرة بالمحاكاة أولاً، مع مدقِّق تباعد لا يشترك في الشيفرة مع المخطِّطات ومختبِر إجهاد",
    skip: "تخطَّ إلى العارض",
    banner_title: "محاكاة فقط",
    banner_en_original: "النص الإنجليزي الأصلي (من index.json):",
    safety_statement:
      "أداة محاكاة وتخطيط فقط. غير معتمدة للطيران ولا تدّعي أي امتثال تنظيمي. تحتاج العروض الحقيقية إلى ملف سلامة تشغيلي وموافقات من الجهات المختصة بالطيران في دولة الإمارات. التحقق تمّ بالمحاكاة فقط.",
    show: "العرض",
    show_meta: "{n} طائرة، {dur} ث، d_min = {dmin} م",
    language: "اللغة",
    lang_switch: "English",
    lang_switch_label: "تبديل اللغة إلى الإنجليزية",
    theme: "السمة",
    theme_dark: "داكنة",
    theme_light: "فاتحة",
    theme_switch_label: "التبديل بين السمة الداكنة والفاتحة",
    methods: "المنهجية",
    methods_title: "كيف يُحسب كل رقم",
    loading: "جارٍ تحميل البيانات…",
    loading_show: "جارٍ تحميل {name}…",
    load_error: "تعذّر تحميل بيانات العارض: {msg}",
    load_hint:
      "يجلب العارض ملفات ثنائية، لذا يجب تشغيله عبر HTTP. من جذر المستودع نفّذ: python -m http.server -d viewer 8000 ثم افتح http://localhost:8000/ (فتح index.html مباشرة عبر file:// لا يعمل).",
    webgl_error: "WebGL غير متاح في هذا المتصفح، لذا لا يمكن رسم العروض ثلاثية الأبعاد.",
    retry: "إعادة المحاولة",

    view_nominal: "الخطة الاسمية",
    view_nominal_badge: "المدقِّق الدقيق",
    view_stress: "سيناريو الإجهاد",
    view_stress_badge: "محاكاة بنماذج مفترضة",
    scenario: "السيناريو",
    canvas_label_a: "عرض ثلاثي الأبعاد للتشكيل الاسمي (الخطة مفحوصة مقابل حدودها المعلنة). اسحب للدوران ومرّر للتكبير.",
    canvas_label_b: "عرض ثلاثي الأبعاد لسيناريو الإجهاد بنفس كاميرا العرض الاسمي. اسحب للدوران ومرّر للتكبير.",
    readout_min_now: "أقل مسافة إلى أقرب جار الآن",
    readout_min_sofar: "أقل مسافة إلى أقرب جار حتى الآن",
    readout_below: "طائرات دون d_min الآن",
    readout_events: "أحداث الأعطال المُفعَّلة",
    unit_m: "م",
    of_n: "{k} من {n}",
    sampled_note: "مأخوذة كل {dt} ث من بيانات العارض؛ قد تفوتها القيم الدنيا بين الإطارات.",
    stress_label: "نتيجة محاكاة بنماذج اضطراب مفترضة. ليست ضمانًا.",
    nominal_label: "قراءات الخطة الاسمية قيم إطارات مأخوذة بالعيّنات؛ الرقم الدقيق في لوحة سجل التحقق.",

    play: "تشغيل",
    pause: "إيقاف مؤقت",
    views_label: "العرضان ثلاثيا الأبعاد: الخطة الاسمية وسيناريو الإجهاد",
    playback_label: "أزرار التشغيل",
    step_back: "إطار إلى الخلف",
    step_fwd: "إطار إلى الأمام",
    restart: "إعادة من البداية",
    speed: "السرعة",
    timeline: "الخط الزمني",
    time_readout: "الزمن {t} من {total} ث",
    colour_mode: "نمط التلوين",
    mode_height: "حسب الارتفاع",
    mode_sep: "حسب التباعد",
    height_legend: "منخفض",
    height_legend_hi: "مرتفع",
    legend_height: "الارتفاع: بنفسجي (الأرض) إلى سماوي فاتح (الأعلى)",
    legend_red: "أحمر: أقل من d_min",
    legend_amber: "كهرماني: أقل من 1.25 d_min",
    legend_green: "أخضر: لا يقل عن ١٫٢٥ d_min (لون عرض وليس تصنيف سلامة)",
    legend_below_big: "الطائرات الأقل من d_min تُرسم أكبر أيضًا",
    legend_geo: "حدود السياج الجغرافي (الأرض والسقف)",
    legend_crowd: "خط الجمهور (متصل)",
    legend_buffer: "خط حاجز الجمهور، crowd.y + أقل مسافة (متقطع)",
    legend_marker_motor: "معيّن: عطل محرّك (سقوط حر)",
    legend_marker_comms: "معيّن: فقدان الاتصال",
    legend_marker_other: "معيّن: حدث آخر",
    legend_size: "تُرسم الطائرات مكبّرة للوضوح (نصف قطر 0.9 م)؛ نصف القطر المنمذج للمركبة 0.24 م.",
    keys_hint: "مفاتيح الخط الزمني: الأسهم ±1 ث، Shift+الأسهم ±إطار، Home/End.",

    cert_title: "سجل التحقق (ليس اعتمادًا ولا شهادة تنظيمية)",
    cert_loading: "جارٍ تحميل سجل التحقق…",
    verdict: "الحكم",
    pass: "نجاح (PASS)",
    fail: "إخفاق (FAIL)",
    verdict_note: "يعني فقط أن المسارات كما كُتبت تحقق الحدود المعلنة في ملف العرض، وبعضها افتراضات. ليس اعتمادًا ولا تصريح سلامة ولا دليلًا على أن عرضًا حقيقيًا سيكون آمنًا.",
    cert_min_sep: "أقل تباعد (دقيق)",
    cert_min_sep_at: "عند t = {t} ث، بين الطائرتين {a} و{b}",
    cert_dmin: "العتبة المعلنة d_min",
    cert_dmin_assumed: "(افتراض)",
    cert_margin: "الهامش فوق d_min",
    cert_recheck: "فحص إعادة الحساب (الشيفرة نفسها)",
    recheck_yes: "نجح: أُعيد حساب البصمة والأرقام بالشيفرة نفسها",
    recheck_no: "لم تنجح",
    cert_sha: "SHA-256 للأسطول",
    copy: "نسخ",
    copied: "تم النسخ",
    copy_failed: "فشل النسخ",
    show_full_hash: "عرض البصمة كاملة",
    cert_pairs: "المقاطع / الأزواج المفحوصة",
    cert_pairs_val: "{seg} مقطعًا، و{pairs} زوج طائرات قريب بما يكفي للحل الدقيق",
    cert_dyn_title: "الحدود الديناميكية (الأقصى المُبلَغ / الحد)",
    dyn_speed_h: "السرعة الأفقية (م/ث)",
    dyn_speed_up: "معدل الصعود (م/ث)",
    dyn_speed_down: "معدل الهبوط (م/ث)",
    dyn_accel: "التسارع (م/ث²)",
    dyn_jerk: "الارتجاج Jerk (م/ث³)",
    dyn_geofence: "هامش السياج الجغرافي (م)",
    dyn_crowd: "هامش الجمهور (م)",
    cert_guarantees: "ما يضمنه المدقِّق (ضمن نموذجه المعلن)",
    cert_not: "ما لا يضمنه",
    cert_statement: "بيان سجل التحقق",
    cert_text_note: "النص مأخوذ من certificate.json (اسم ملف فقط؛ المحتوى سجل تحقق وليس اعتمادًا)، مع ترجمة عربية للبنود المعروفة.",
    cert_error: "تعذّر تحميل الشهادة: {msg}",

    scen_title: "تفاصيل السيناريو",
    scen_assumed: "هذه نتائج محاكاة بنماذج مفترضة (معاملات الرياح وGNSS والأعطال افتراضات، انظر المنهجية). ليست ضمانات وليست قياسات.",
    scen_scope: "النطاق: تُقيَّم هنا المسافات بين الطائرات فقط. لا يُقيَّم تحت الاضطراب البقاء داخل السياج الجغرافي ولا البعد عن الجمهور ولا أثر سقوط الطائرات على الأرض ولا تحمّل البطارية ولا الإقلاع والهبوط.",
    scen_min_nn: "أقل مسافة إلى أقرب جار خلال التشغيل",
    scen_max_err: "أقصى خطأ تتبّع",
    scen_seed: "البذرة العشوائية (تحقّق واحد ثابت)",
    scen_config: "معاملات السيناريو (مفترضة)",
    scen_events: "الأحداث",
    scen_no_events: "لا توجد أحداث أعطال منفصلة في هذا السيناريو.",
    scen_ballistic: "طائرات في سقوط حر (غير متحكَّم به)",
    scen_none: "لا يوجد",
    jump: "انتقل إلى",
    event_row: "الطائرة {d}: {kind} عند {t} ث",
    kind_motor_failure: "عطل محرّك",
    kind_comms_loss: "فقدان الاتصال",
    kind_battery_low: "بطارية منخفضة",
    kind_gnss_loss: "فقدان GNSS",
    cfg_w20: "متوسط الرياح عند 20 قدمًا (م/ث)",
    cfg_gnss_deg_fraction: "نسبة الطائرات ذات GNSS المتدهور",
    cfg_gnss_bias_sigma: "انحراف GNSS المعياري للتحيّز (م)",
    cfg_motor_failures: "عدد أعطال المحرّكات",
    cfg_comms_loss_fraction: "نسبة الطائرات التي تفقد الاتصال",
    cfg_behavior: "سلوك الأعطال",
    scen_nominal_note: "يعرض العرض الاسمي الخطة نفسها كما اجتازت فحص الحدود المعلنة، دون أي اضطراب.",

    foot_note: "سرب: المحاكاة أولاً. غير معتمد للطيران. مكتبة three.js (رخصة MIT) مضمّنة في vendor/.",
    m_back: "العودة إلى العارض",
    m_title: "المنهجية: كيف يُحسب كل رقم في العارض",
    m_toc: "المحتويات",
    m_lang_note: "يقدّم النص العربي العنوان وملخصًا قصيرًا لكل قسم؛ والنص الإنجليزي المفصّل هو المرجع. تبقى المعرّفات التقنية بالأحرف اللاتينية.",
  },
};

// Arabic renderings of the fixed certificate text (sirb/certificate.py GUARANTEES / NOT_GUARANTEED).
// Used only when the certificate has the same number of items; otherwise the English is shown.
export const CERT_AR = {
  guarantees: [
    "ضمن النموذج المعلن (مسارات متعددة الحدود كما كُتبت تمامًا، وطائرات كنقاط تتطلب تباعدًا min_separation_m بين المراكز)، يكون أقل تباعد مُبلَّغ عنه على امتداد الزمن دقيقًا حتى حدود الخطأ العددي وخطأ إيجاد الجذور، لكل زوج يقل أدنى تباعد له عن نصف قطر البحث.",
    "تتحقق الحدود الديناميكية (السرعة، معدل الصعود والهبوط، التسارع، الارتجاج، الارتفاع، السياج الجغرافي، المسافة إلى الجمهور) للمتعددات الحدود كما كُتبت، ضمن سياسة التسامح المعلنة.",
    "المقاطع متصلة زمنيًا مع استمرارية الموضع والسرعة والتسارع ضمن التسامحات المعلنة (قفزة في التسارع تعني ارتجاجًا غير محدود).",
  ],
  not_guaranteed: [
    "تتبّع الطائرات الحقيقية للمسارات المأمورة (خطأ المتحكم، التأخير)",
    "الرياح وسائر الاضطرابات الجوية",
    "خطأ موضع GNSS أو التشويش أو الانتحال",
    "التداخل الهوائي مثل التدفق الهابط (downwash)",
    "حجم المركبة وشكلها أو سلوك البطارية خارج الحدود المعلنة",
    "الأعطال وفقدان الاتصال وسلوكيات الطوارئ",
    "أي خصائص تنظيمية أو خاصة بصلاحية الطيران أو الموافقات التشغيلية",
  ],
  statement: "تحقّق بالمحاكاة فقط؛ غير معتمد للطيران؛ لا ادّعاء بالامتثال التنظيمي",
};

const KEY = "sirb.viewer.lang";
export function getLang() {
  try {
    const v = localStorage.getItem(KEY);
    if (v === "en" || v === "ar") return v;
  } catch (e) { /* storage unavailable */ }
  return "en";
}
export function storeLang(lang) {
  try { localStorage.setItem(KEY, lang); } catch (e) { /* ignore */ }
}
export function applyDocLang(lang) {
  document.documentElement.lang = lang;
  document.documentElement.dir = lang === "ar" ? "rtl" : "ltr";
}
export function makeT(lang) {
  return (key, vars) => {
    let s = (STRINGS[lang] && STRINGS[lang][key]) ?? STRINGS.en[key] ?? key;
    if (vars) for (const [k, v] of Object.entries(vars)) s = s.split(`{${k}}`).join(String(v));
    return s;
  };
}
// Number formatting; Arabic uses Arabic-Indic digits.
const cache = {};
export function makeNum(lang) {
  return (v, digits = 2) => {
    if (v === null || v === undefined || Number.isNaN(v)) return "—";
    const k = lang + digits;
    cache[k] ??= new Intl.NumberFormat(lang === "ar" ? "ar-u-nu-arab" : "en", {
      minimumFractionDigits: digits, maximumFractionDigits: digits, useGrouping: false });
    return cache[k].format(v);
  };
}
export function makeInt(lang) {
  const f = new Intl.NumberFormat(lang === "ar" ? "ar-u-nu-arab" : "en", { useGrouping: false });
  return (v) => f.format(v);
}
// Apply data-i18n / data-i18n-aria / data-i18n-title attributes under root.
export function translateDom(root, t) {
  root.querySelectorAll("[data-i18n]").forEach((el) => { el.textContent = t(el.dataset.i18n); });
  root.querySelectorAll("[data-i18n-aria]").forEach((el) => el.setAttribute("aria-label", t(el.dataset.i18nAria)));
  root.querySelectorAll("[data-i18n-title]").forEach((el) => el.setAttribute("title", t(el.dataset.i18nTitle)));
}
