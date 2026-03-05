async function fetchJson(url, options = {}, allowErrorResponse = false) {
  const res = await fetch(url, options);
  const body = await res.json();
  if (!allowErrorResponse && (!res.ok || body.ok === false)) {
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
  const rehearsal = state.rehearsal || {};
  const rehText = rehearsal.active
    ? `${rehearsal.label || "unnamed"} (${rehearsal.profile_id || "profile?"})`
    : "inactive";
  document.getElementById("rehearsalSessionState").textContent = rehText;
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

function renderPreflight(preflight) {
  if (!preflight || !Object.keys(preflight).length) {
    document.getElementById("preflightStatus").textContent = "not run";
    document.getElementById("preflightDetails").textContent = "Run preflight to populate checklist.";
    return;
  }
  const status = preflight.ok
    ? `pass (${preflight.required_failures || 0} fail / ${preflight.warnings || 0} warn)`
    : `fail (${preflight.required_failures || 0} fail / ${preflight.warnings || 0} warn)`;
  document.getElementById("preflightStatus").textContent = status;
  const lines = (preflight.checks || []).map((c) => `[${c.level}] ${c.message}`);
  document.getElementById("preflightDetails").textContent = lines.join("\n") || "No checks parsed";
}

async function refreshRehearsalProfiles() {
  const { profiles } = await fetchJson("/api/rehearsal/profiles");
  const select = document.getElementById("rehearsalProfile");
  const previous = select.value;
  select.innerHTML = "";
  profiles.forEach((p) => {
    const opt = document.createElement("option");
    opt.value = p.id;
    opt.textContent = p.name;
    if (p.description) opt.title = p.description;
    select.appendChild(opt);
  });
  if (previous) select.value = previous;
}

async function refreshRehearsalSession() {
  const { rehearsal } = await fetchJson("/api/rehearsal/session");
  const text = rehearsal.active
    ? `${rehearsal.label || "unnamed"} (${rehearsal.profile_id || "profile?"})`
    : "inactive";
  document.getElementById("rehearsalSessionState").textContent = text;
  renderPreflight(rehearsal.last_preflight || {});
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

async function runPreflight() {
  const body = await fetchJson(
    "/api/rehearsal/preflight",
    {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({}),
    },
    true,
  );
  renderPreflight(body.preflight || {});
  await refreshRehearsalSession();
}

async function startRehearsal() {
  const profile_id = document.getElementById("rehearsalProfile").value;
  const labelInput = document.getElementById("rehearsalLabel");
  const notes = document.getElementById("rehearsalNotes").value || "";
  const body = await fetchJson(
    "/api/rehearsal/start",
    {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ profile_id, label: labelInput.value || "", notes }),
    },
    true,
  );
  if (!body.ok) {
    renderPreflight(body.preflight || {});
    throw new Error(body.error || "Rehearsal start failed");
  }
  if (body.rehearsal && body.rehearsal.label) {
    labelInput.value = body.rehearsal.label;
  }
  renderPreflight(body.preflight || {});
  await Promise.all([
    refreshState(),
    refreshSupervisor(),
    refreshRuntimeHealth(),
    refreshRehearsalSession(),
  ]);
}

async function stopRehearsal() {
  await fetchJson("/api/rehearsal/stop", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({}),
  });
  await Promise.all([
    refreshState(),
    refreshSupervisor(),
    refreshRuntimeHealth(),
    refreshRehearsalSession(),
  ]);
}

function runAction(action) {
  action().catch((err) => {
    console.error(err);
    alert(err.message || String(err));
  });
}

function wireEvents() {
  document.getElementById("applyRecipe").addEventListener("click", () => runAction(applyRecipe));
  document.getElementById("consentOn").addEventListener("click", () => runAction(() => setConsent(1)));
  document.getElementById("consentOff").addEventListener("click", () => runAction(() => setConsent(0)));
  document.getElementById("startRuntime").addEventListener("click", () => runAction(startRuntime));
  document.getElementById("stopRuntime").addEventListener("click", () => runAction(stopRuntime));
  document.getElementById("exportSession").addEventListener("click", () => runAction(exportSession));
  document.getElementById("runPreflight").addEventListener("click", () => runAction(runPreflight));
  document.getElementById("startRehearsal").addEventListener("click", () => runAction(startRehearsal));
  document.getElementById("stopRehearsal").addEventListener("click", () => runAction(stopRehearsal));
}

async function boot() {
  wireEvents();
  await Promise.all([refreshRecipes(), refreshRehearsalProfiles()]);
  await Promise.all([
    refreshState(),
    refreshCurves(),
    refreshRuntimeHealth(),
    refreshSupervisor(),
    refreshRehearsalSession(),
  ]);
  setInterval(() => refreshState().catch(console.error), 1000);
  setInterval(() => refreshCurves().catch(console.error), 2500);
  setInterval(() => refreshRuntimeHealth().catch(console.error), 3000);
  setInterval(() => refreshSupervisor().catch(console.error), 2000);
  setInterval(() => refreshRehearsalSession().catch(console.error), 3000);
}

boot().catch((err) => {
  console.error(err);
  alert(`Operator UI failed to initialize: ${err.message}`);
});
