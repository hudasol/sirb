// Classic (non-module) script: modules cannot load from file:// (CORS), so app.js never runs there.
// This reveals a static bilingual notice instead of leaving a blank page.
if (location.protocol === "file:") {
  var b = document.getElementById("fileBanner");
  if (b) b.hidden = false;
}
