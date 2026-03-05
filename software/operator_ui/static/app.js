async function fetchJson(url, options = {}) {
  const res = await fetch(url, options);
  const body = await res.json();
  if (!res.ok || body.ok === false) {
    throw new Error(body.error || `Request failed: ${res.status}`);
  }
  return body;
}

function drawCurve(canvas, series, color) {
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  ctx.fillStyle = "#fff";
  ctx.fillRect(0, 0, w, h);

  ctx.strokeStyle = "#d8cfbe";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, h / 2);
  ctx.lineTo(w, h / 2);
  ctx.moveTo(w / 2, 0);
  ctx.lineTo(w / 2, h);
  ctx.stroke();

  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  series.forEach((p, idx) => {
    const x = ((p.x + 1) / 2) * w;
    const y = h - ((p.y + 1) / 2) * h;
    if (idx === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

async function refreshState() {
  const { state } = await fetchJson("/api/state");
  document.getElementById("activeRecipe").textContent = state.active_recipe;
  document.getElementById("consentState").textContent = state.consent_state ? "ON" : "OFF";
  document.getElementById("oscPort").textContent = state.osc_port;
  document.getElementById("consentMode").textContent = state.consent_mode;
  document.getElementById("runtimeTargets").textContent = state.runtime_targets.join(", ") || "none";
  const dispatch = state.last_dispatch || {};
  const results = dispatch.results || [];
  const okCount = results.filter((r) => r.ok).length;
  const status = dispatch.action && dispatch.action !== "none"
    ? `${dispatch.action} (${okCount}/${results.length} ok)`
    : "none";
  document.getElementById("lastDispatch").textContent = status;
  document.getElementById("latestSession").textContent = state.last_export || "none";
}

async function refreshRuntimeHealth() {
  const { runtime } = await fetchJson("/api/runtime/health");
  document.getElementById("runtimeHealthSummary").textContent = `${runtime.healthy}/${runtime.total} healthy`;
  const details = (runtime.services || []).map((svc) => {
    const marker = svc.healthy ? "OK" : "DOWN";
    const pid = svc.pid ? ` pid=${svc.pid}` : "";
    return `${marker} ${svc.name}${pid} (${svc.source}) - ${svc.detail}`;
  });
  document.getElementById("runtimeHealthDetails").textContent = details.join("\n") || "No services configured";
}

async function refreshSupervisor() {
  const { supervisor } = await fetchJson("/api/runtime/supervisor");
  const state = supervisor.running
    ? `running (pid ${supervisor.pid})`
    : `stopped (exit ${supervisor.last_exit_code})`;
  document.getElementById("supervisorState").textContent = state;
}

async function refreshRecipes() {
  const { recipes } = await fetchJson("/api/recipes");
  const select = document.getElementById("recipeSelect");
  const previous = select.value;
  select.innerHTML = "";
  recipes.forEach((r) => {
    const opt = document.createElement("option");
    opt.value = r.id;
    opt.textContent = r.name;
    if (r.description) opt.title = r.description;
    select.appendChild(opt);
  });
  if (previous) select.value = previous;
}

async function refreshCurves() {
  const { data } = await fetchJson("/api/mapping/curves?points=121");
  drawCurve(document.getElementById("altCurve"), data.curves.altitude.series, "#1f6f78");
  drawCurve(document.getElementById("latCurve"), data.curves.lateral.series, "#c44536");
}

async function applyRecipe() {
  const recipe = document.getElementById("recipeSelect").value;
  await fetchJson("/api/recipe", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ recipe }),
  });
  await refreshState();
  await refreshCurves();
}

async function setConsent(value) {
  await fetchJson("/api/consent", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ consent: value }),
  });
  await refreshState();
}

async function exportSession() {
  const label = document.getElementById("sessionLabel").value || "";
  const { session } = await fetchJson("/api/session/export", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ label }),
  });
  document.getElementById("latestSession").textContent = session.path;
  await refreshState();
}

async function startRuntime() {
  await fetchJson("/api/runtime/start", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({}),
  });
  await Promise.all([refreshSupervisor(), refreshRuntimeHealth()]);
}

async function stopRuntime() {
  await fetchJson("/api/runtime/stop", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({}),
  });
  await Promise.all([refreshSupervisor(), refreshRuntimeHealth()]);
}

function wireEvents() {
  document.getElementById("applyRecipe").addEventListener("click", applyRecipe);
  document.getElementById("consentOn").addEventListener("click", () => setConsent(1));
  document.getElementById("consentOff").addEventListener("click", () => setConsent(0));
  document.getElementById("startRuntime").addEventListener("click", startRuntime);
  document.getElementById("stopRuntime").addEventListener("click", stopRuntime);
  document.getElementById("exportSession").addEventListener("click", exportSession);
}

async function boot() {
  wireEvents();
  await refreshRecipes();
  await Promise.all([refreshState(), refreshCurves(), refreshRuntimeHealth(), refreshSupervisor()]);
  setInterval(() => refreshState().catch(console.error), 1000);
  setInterval(() => refreshCurves().catch(console.error), 2500);
  setInterval(() => refreshRuntimeHealth().catch(console.error), 3000);
  setInterval(() => refreshSupervisor().catch(console.error), 2000);
}

boot().catch((err) => {
  console.error(err);
  alert(`Operator UI failed to initialize: ${err.message}`);
});
