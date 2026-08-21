async function fetchJson(url, options = {}, allowErrorResponse = false) {
  const token = localStorage.getItem("operatorToken");
  if (token) {
    options.headers = options.headers || {};
    options.headers["Authorization"] = `Bearer ${token}`;
  }
  const res = await fetch(url, options);
  if (res.status === 401) {
    const input = prompt("API token required for this action. Enter token:");
    if (input) {
      localStorage.setItem("operatorToken", input);
      return fetchJson(url, options, allowErrorResponse);
    } else {
      throw new Error("unauthorized (token required)");
    }
  }
  const body = await res.json();
  if (!allowErrorResponse && (!res.ok || body.ok === false)) {
    throw new Error(body.error || `Request failed: ${res.status}`);
  }
  return body;
}

const showModeState = {
  consentOn: null,
  latestExport: "",
  rehearsalActive: false,
  rehearsalProfile: "",
  rehearsalLabel: "",
  preflightCheckedAt: 0,
  preflightOk: null,
  runtimeHealthy: 0,
  runtimeTotal: 0,
};

function clearChipState(el) {
  el.classList.remove("ok", "warn", "bad");
}

function updateChip(el, text, level = "") {
  if (!el) return;
  el.textContent = text;
  clearChipState(el);
  if (level) {
    el.classList.add(level);
  }
}

function relativeAge(seconds) {
  if (!seconds || seconds <= 0) return "n/a";
  if (seconds < 60) return `${Math.floor(seconds)}s ago`;
  if (seconds < 3600) return `${Math.floor(seconds / 60)}m ago`;
  return `${Math.floor(seconds / 3600)}h ago`;
}

function renderShowStrip() {
  const now = Date.now() / 1000;

  const preflightChip = document.getElementById("showPreflight");
  if (!showModeState.preflightCheckedAt) {
    updateChip(preflightChip, "Preflight: not run");
  } else {
    const age = Math.max(0, now - showModeState.preflightCheckedAt);
    const freshness = age <= 600 ? "fresh" : (age <= 1800 ? "aging" : "stale");
    const level = showModeState.preflightOk
      ? (freshness === "fresh" ? "ok" : "warn")
      : "bad";
    updateChip(
      preflightChip,
      `Preflight: ${showModeState.preflightOk ? "pass" : "fail"} (${freshness}, ${relativeAge(age)})`,
      level,
    );
  }

  const profileText = showModeState.rehearsalProfile
    ? showModeState.rehearsalProfile
    : "none";
  const profileLevel = showModeState.rehearsalActive ? "ok" : "";
  const profileChip = document.getElementById("showProfile");
  const profileSession = showModeState.rehearsalActive && showModeState.rehearsalLabel
    ? ` / ${showModeState.rehearsalLabel}`
    : "";
  updateChip(profileChip, `Profile: ${profileText}${profileSession}`, profileLevel);

  const consentChip = document.getElementById("showConsent");
  if (showModeState.consentOn == null) {
    updateChip(consentChip, "Consent: n/a");
  } else {
    updateChip(
      consentChip,
      `Consent: ${showModeState.consentOn ? "ON" : "OFF"}`,
      showModeState.consentOn ? "ok" : "bad",
    );
  }

  const runtimeChip = document.getElementById("showRuntimeHealth");
  if (!showModeState.runtimeTotal) {
    updateChip(runtimeChip, "Runtime: n/a");
  } else {
    const level = showModeState.runtimeHealthy === showModeState.runtimeTotal
      ? "ok"
      : (showModeState.runtimeHealthy > 0 ? "warn" : "bad");
    updateChip(
      runtimeChip,
      `Runtime: ${showModeState.runtimeHealthy}/${showModeState.runtimeTotal} healthy`,
      level,
    );
  }

  const exportChip = document.getElementById("showLatestExport");
  if (!showModeState.latestExport) {
    updateChip(exportChip, "Export: none", "warn");
    return;
  }
  const parts = showModeState.latestExport.split("/");
  const basename = parts[parts.length - 1] || showModeState.latestExport;
  updateChip(exportChip, `Export: ${basename}`, "ok");
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

function macroDisplayValue(macro, value) {
  return macro.type === "bipolar" ? `${value >= 0 ? "+" : ""}${value.toFixed(2)}` : value.toFixed(2);
}

const macroStreams = {};
const MACRO_STREAM_INTERVAL_MS = 40;

async function sendMacro(macroId, value) {
  await fetchJson("/api/performance", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ values: { [macroId]: value } }),
  });
}

function macroStream(macroId) {
  if (!macroStreams[macroId]) {
    macroStreams[macroId] = { lastSentAt: 0, pending: null, timer: null };
  }
  return macroStreams[macroId];
}

function transmitMacro(macroId, value) {
  const stream = macroStream(macroId);
  stream.lastSentAt = Date.now();
  stream.pending = null;
  runAction(() => sendMacro(macroId, value));
}

function flushMacro(macroId) {
  const stream = macroStream(macroId);
  stream.timer = null;
  if (stream.pending !== null) transmitMacro(macroId, stream.pending);
}

function streamMacro(macroId, value, final = false) {
  const stream = macroStream(macroId);
  if (final) {
    if (stream.timer) clearTimeout(stream.timer);
    stream.timer = null;
    transmitMacro(macroId, value);
    return;
  }

  const elapsed = Date.now() - stream.lastSentAt;
  if (elapsed >= MACRO_STREAM_INTERVAL_MS && !stream.timer) {
    transmitMacro(macroId, value);
    return;
  }

  stream.pending = value;
  if (!stream.timer) {
    stream.timer = setTimeout(
      () => flushMacro(macroId),
      Math.max(0, MACRO_STREAM_INTERVAL_MS - elapsed),
    );
  }
}

async function refreshPerformance() {
  const { performance } = await fetchJson("/api/performance");
  const root = document.getElementById("macroControls");
  root.innerHTML = "";
  (performance.macros || []).forEach((macro) => {
    const value = Number((performance.targets || {})[macro.id] ?? macro.default);
    const wrap = document.createElement("div");
    wrap.className = "macro";
    const min = macro.type === "bipolar" ? -1 : 0;
    wrap.innerHTML = `<label>${macro.label}<span class="value">${macroDisplayValue(macro, value)}</span></label><small>${macro.description || ""}</small>`;
    const input = document.createElement("input");
    input.type = "range"; input.min = min; input.max = 1; input.step = "0.01"; input.value = value;
    const unavailable = (macro.requires || []).length > 0;
    if (unavailable) {
      input.disabled = true;
      wrap.querySelector("small").textContent = `Unavailable — requires ${macro.requires.join(", ")}`;
    }
    input.addEventListener("input", () => {
      const next = Number(input.value);
      wrap.querySelector(".value").textContent = macroDisplayValue(macro, next);
      streamMacro(macro.id, next);
    });
    input.addEventListener("change", () => {
      const next = Number(input.value);
      streamMacro(macro.id, next, true);
      runAction(refreshCurves);
    });
    wrap.appendChild(input); root.appendChild(wrap);
  });
}

async function refreshRecipeCards() {
  const { recipes } = await fetchJson("/api/recipes");
  const { state } = await fetchJson("/api/state");
  const root = document.getElementById("recipeCards"); root.innerHTML = "";
  recipes.forEach((recipe) => {
    const card = document.createElement("button"); card.className = `recipe-card ${state.active_recipe === recipe.id ? "active" : ""}`;
    card.innerHTML = `<strong>${recipe.name}</strong><small>${recipe.intent || recipe.description || "A versioned scene."}</small>`;
    card.addEventListener("click", () => runAction(async () => { await fetchJson("/api/recipe", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ recipe: recipe.id }) }); await Promise.all([refreshState(), refreshRecipeCards(), refreshPerformance(), refreshCurves()]); }));
    root.appendChild(card);
  });
}

async function refreshLiveFlow() {
  const { data } = await fetchJson("/api/live-flow"); const flow = data.flow;
  const root = document.getElementById("signalFlow");
  if (!data.available || !flow) {
    root.textContent = data.reason === "stale"
      ? `TRAINER TELEMETRY LOST — last update ${Number(data.age || 0).toFixed(1)}s ago`
      : "Waiting for trainer telemetry…";
    return;
  }
  const stages = [["Body input", flow.input], ["Intent", flow.intent], ["Safe output", flow.safe_output]];
  const observed = data.performance || {};
  const observedText = observed.targets
    ? `Observed ${Object.entries(observed.targets).map(([id, value]) => `${id} ${Number(value).toFixed(2)}`).join(" · ")}`
    : "Observed macro state unavailable";
  root.innerHTML = stages.map(([name, values], idx) => `<div class="flow-stage ${idx === 2 && flow.limited ? "limited" : ""}"><span>${name}</span><strong>roll ${Number(values.roll || 0).toFixed(2)}</strong><br>yaw ${Number(values.yaw || 0).toFixed(2)}</div>`).join("") + `<small>${observedText}</small>`;
}

function selectMode(mode) {
  document.querySelectorAll("[data-view]").forEach((section) => {
    section.classList.toggle("hidden", section.dataset.view !== mode);
  });
  document.querySelectorAll(".mode-tab").forEach((tab) => {
    tab.classList.toggle("active", tab.dataset.mode === mode);
  });
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
  showModeState.consentOn = !!state.consent_state;
  showModeState.latestExport = state.last_export || "";
  const rehearsal = state.rehearsal || {};
  showModeState.rehearsalActive = !!rehearsal.active;
  showModeState.rehearsalProfile = rehearsal.profile_id || "";
  showModeState.rehearsalLabel = rehearsal.label || "";
  const rehText = rehearsal.active
    ? `${rehearsal.label || "unnamed"} (${rehearsal.profile_id || "profile?"})`
    : "inactive";
  document.getElementById("rehearsalSessionState").textContent = rehText;
  renderShowStrip();
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
  showModeState.runtimeHealthy = Number(runtime.healthy || 0);
  showModeState.runtimeTotal = Number(runtime.total || 0);
  renderShowStrip();
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
  const preflight = rehearsal.last_preflight || {};
  showModeState.preflightCheckedAt = Number(preflight.checked_at || 0);
  showModeState.preflightOk = preflight.ok == null ? null : !!preflight.ok;
  showModeState.rehearsalProfile = rehearsal.profile_id || showModeState.rehearsalProfile;
  showModeState.rehearsalLabel = rehearsal.label || showModeState.rehearsalLabel;
  showModeState.rehearsalActive = !!rehearsal.active;
  renderShowStrip();
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
  document.querySelectorAll(".mode-tab").forEach((tab) => {
    tab.addEventListener("click", () => selectMode(tab.dataset.mode));
  });
  document.getElementById("applyRecipe").addEventListener("click", () => runAction(applyRecipe));
  document.getElementById("consentOn").addEventListener("click", () => runAction(() => setConsent(1)));
  document.getElementById("consentOff").addEventListener("click", () => runAction(() => setConsent(0)));
  document.getElementById("startRuntime").addEventListener("click", () => runAction(startRuntime));
  document.getElementById("stopRuntime").addEventListener("click", () => runAction(stopRuntime));
  document.getElementById("exportSession").addEventListener("click", () => runAction(exportSession));
  document.getElementById("runPreflight").addEventListener("click", () => runAction(runPreflight));
  document.getElementById("startRehearsal").addEventListener("click", () => runAction(startRehearsal));
  document.getElementById("stopRehearsal").addEventListener("click", () => runAction(stopRehearsal));
  document.getElementById("resetPerformance").addEventListener("click", () => runAction(async () => { await fetchJson("/api/performance/reset", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" }); await Promise.all([refreshPerformance(), refreshCurves()]); }));
  document.getElementById("playExport").addEventListener("click", () => runAction(exportSession));
}

async function boot() {
  wireEvents();
  selectMode("play");
  renderShowStrip();
  await Promise.all([refreshRecipes(), refreshRecipeCards(), refreshPerformance(), refreshRehearsalProfiles()]);
  await Promise.all([
    refreshState(),
    refreshCurves(),
    refreshRuntimeHealth(),
    refreshSupervisor(),
    refreshRehearsalSession(),
    refreshLiveFlow(),
  ]);
  setInterval(() => refreshState().catch(console.error), 1000);
  setInterval(() => refreshCurves().catch(console.error), 2500);
  setInterval(() => refreshRuntimeHealth().catch(console.error), 3000);
  setInterval(() => refreshSupervisor().catch(console.error), 2000);
  setInterval(() => refreshRehearsalSession().catch(console.error), 3000);
  setInterval(() => refreshLiveFlow().catch(console.error), 500);
}

boot().catch((err) => {
  console.error(err);
  alert(`Operator UI failed to initialize: ${err.message}`);
});
