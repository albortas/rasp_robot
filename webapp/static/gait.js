/* Tuning interactivo de la marcha del Robodelta.
 *
 * Flujo:
 *   1. GET /api/gait/state  -> limites y valores por defecto
 *   2. POST /api/gait/preview -> frames simulados (pies + angulos IK)
 *   3. El canvas anima esos frames y permite orbitar la camara.
 *   4. POST /api/gait/start / stop -> ejecucion real con el hardware.
 */

"use strict";

// ------------------------------------------------------------------ constantes
const LEG_ORDER = ["FL", "FR", "RL", "RR"];
const LEG_COLORS = {
  FL: "#38bdf8",
  FR: "#f59e0b",
  RL: "#22c55e",
  RR: "#f472b6",
};
const LEG_NAMES = { FL: "Frontal izq.", FR: "Frontal der.", RL: "Trasera izq.", RR: "Trasera der." };

const FALLBACK_LIMITS = {
  x: [-0.04, 0.04],
  y: [-0.04, 0.04],
  z: [-0.03, 0.03],
  roll: [-15, 15],
  pitch: [-15, 15],
  yaw: [-15, 15],
  L: [0.0, 0.06],
  vel: [0.0, 0.15],
  lateral: [-90, 90],
  yaw_rate: [-30, 30],
  clearance_height: [0.01, 0.08],
  penetration_depth: [0.0, 0.02],
  Tswing: [0.15, 0.4],
  max_stance_ratio: [1.0, 5.0],
};

const FALLBACK_DEFAULTS = {
  x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0,
  L: 0.025, vel: 0.05, lateral: 0, yaw_rate: 0,
  clearance_height: 0.04, penetration_depth: 0.005, Tswing: 0.25,
  max_stance_ratio: 1.3,
};

const GROUPS = [
  {
    title: "Postura del cuerpo (IK)",
    controls: [
      { key: "x", label: "X · adelante", min: -0.04, max: 0.04, step: 0.001, dec: 3, unit: "m", hint: "+ = adelante" },
      { key: "y", label: "Y · lateral", min: -0.04, max: 0.04, step: 0.001, dec: 3, unit: "m", hint: "+ = izquierda" },
      { key: "z", label: "Z · altura del cuerpo", min: -0.03, max: 0.03, step: 0.001, dec: 3, unit: "m", hint: "+ = arriba" },
      { key: "roll", label: "Roll", min: -15, max: 15, step: 0.5, dec: 1, unit: "°" },
      { key: "pitch", label: "Pitch", min: -15, max: 15, step: 0.5, dec: 1, unit: "°" },
      { key: "yaw", label: "Yaw", min: -15, max: 15, step: 0.5, dec: 1, unit: "°" },
    ],
  },
  {
    title: "Locomoción",
    controls: [
      { key: "L", label: "L · semizancada", min: 0, max: 0.06, step: 0.0005, dec: 3, unit: "m", hint: "la zancada es 2L" },
      { key: "vel", label: "vel · velocidad de referencia", min: 0, max: 0.15, step: 0.001, dec: 3, unit: "m/s", hint: "solo fija Tstance = 2L / vel" },
      { key: "lateral", label: "LateralFraction", min: -90, max: 90, step: 1, dec: 0, unit: "°", hint: "0° adelante · +90° izquierda · −90° derecha" },
      { key: "yaw_rate", label: "YawRate", min: -30, max: 30, step: 0.5, dec: 1, unit: "°/s", hint: "+ = giro antihorario" },
    ],
    extra: "link-L",
  },
  {
    title: "Contacto y tiempo",
    controls: [
      { key: "clearance_height", label: "Altura de despeje", min: 0.01, max: 0.08, step: 0.001, dec: 3, unit: "m", hint: "altura máxima del pie en vuelo" },
      { key: "penetration_depth", label: "Profundidad de apoyo", min: 0, max: 0.02, step: 0.0005, dec: 4, unit: "m", hint: "cuánto baja el pie en apoyo" },
      { key: "Tswing", label: "Tswing · duración del vuelo", min: 0.15, max: 0.4, step: 0.005, dec: 3, unit: "s", hint: "periodo = Tstance + Tswing" },
      { key: "max_stance_ratio", label: "Límite Tstance / Tswing", min: 1.0, max: 5.0, step: 0.1, dec: 1, unit: "×", hint: "1.3 = estable · >1.5 experimental" },
    ],
  },
];

// ------------------------------------------------------------------- estado
let serverState = null;
let params = { ...FALLBACK_DEFAULTS };
let preview = null;
let previewSeq = 0;
let previewTimer = null;

let frameIdx = 0;
let playing = true;
let speed = 0.5;
let scrubbing = false;
let lastRaf = null;
let lastShown = -1;

let running = false;
let lastRunError = null;

const cam = { yaw: -0.55, pitch: 0.62, zoom: 1 };

const els = {};
const controlInputs = new Map();

// ------------------------------------------------------------------- helpers
const $ = (sel) => document.querySelector(sel);

async function api(path, body) {
  const method = body === undefined ? "GET" : "POST";
  const res = await fetch(path, {
    method,
    headers: body === undefined ? undefined : { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`);
  return data;
}

function toast(message, isError = false) {
  const el = els.toast;
  el.textContent = message;
  el.classList.toggle("error", isError);
  el.classList.remove("hidden");
  clearTimeout(toast._t);
  toast._t = setTimeout(() => el.classList.add("hidden"), 3200);
}

function fmt(value, dec, unit = "") {
  const n = Number(value);
  if (!Number.isFinite(n)) return "—";
  return `${n.toFixed(dec)}${unit ? " " + unit : ""}`;
}

function escapeHtml(text) {
  return String(text).replace(/[&<>"']/g, (c) => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;",
  })[c]);
}

// -------------------------------------------------------------------- badge
function updateModeBadge() {
  const real = serverState && serverState.hardware === "real";
  els.modeBadge.textContent = real ? "MODO REAL" : "SIMULACIÓN";
  els.modeBadge.className = `badge ${real ? "real" : "sim"}`;
  els.modeBadge.title = (serverState && serverState.error) || "";
  els.btnApply.textContent = real ? "Ejecutar en robot" : "Ejecutar (simulación)";
}

// ----------------------------------------------------------------- controles
function buildControls() {
  const limits = (serverState && serverState.limits) || FALLBACK_LIMITS;
  const container = els.groups;
  container.innerHTML = "";

  for (const group of GROUPS) {
    const section = document.createElement("section");
    section.className = "ctrl-group";
    const title = document.createElement("h3");
    title.textContent = group.title;
    section.appendChild(title);

    for (const spec of group.controls) {
      const lim = limits[spec.key] || [spec.min, spec.max];
      const min = Math.min(lim[0], lim[1]);
      const max = Math.max(lim[0], lim[1]);

      const row = document.createElement("div");
      row.className = "ctrl";

      const label = document.createElement("label");
      label.textContent = spec.label;
      label.htmlFor = `gait-${spec.key}`;

      const out = document.createElement("output");
      out.dataset.out = spec.key;

      const input = document.createElement("input");
      input.type = "range";
      input.id = `gait-${spec.key}`;
      input.min = min;
      input.max = max;
      input.step = spec.step;
      input.value = params[spec.key];
      input.dataset.key = spec.key;

      row.append(label, out, input);
      if (spec.hint) {
        const hint = document.createElement("div");
        hint.className = "hint";
        hint.textContent = spec.hint;
        row.appendChild(hint);
      }
      section.appendChild(row);

      const updateOut = () => {
        out.textContent = fmt(params[spec.key], spec.dec, spec.unit);
      };

      input.addEventListener("input", () => {
        params[spec.key] = parseFloat(input.value);
        updateOut();
        schedulePreview();
      });

      controlInputs.set(spec.key, { input, out, spec, updateOut });
      updateOut();
    }

    if (group.extra === "link-L") {
      const btn = document.createElement("button");
      btn.type = "button";
      btn.className = "link-btn";
      btn.textContent = "Ajustar L = vel / 2";
      btn.addEventListener("click", () => {
        const entry = controlInputs.get("L");
        const value = Math.min(params.vel / 2, Number(entry.input.max));
        params.L = value;
        entry.input.value = String(value);
        entry.updateOut();
        schedulePreview();
      });
      section.appendChild(btn);
    }

    container.appendChild(section);
  }
}

function syncControls() {
  for (const [key, entry] of controlInputs) {
    entry.input.value = String(params[key]);
    entry.updateOut();
  }
}

// ------------------------------------------------------------------ preview
function schedulePreview() {
  clearTimeout(previewTimer);
  previewTimer = setTimeout(refreshPreview, 110);
}

async function refreshPreview() {
  const seq = ++previewSeq;
  els.vizSub.textContent = "Calculando simulación…";
  try {
    const data = await api("/api/gait/preview", params);
    if (seq !== previewSeq) return; // respuesta vieja
    preview = data;
    if (!Number.isFinite(frameIdx) || frameIdx >= data.frames) frameIdx = 0;
    els.scrub.max = String(Math.max(0, data.frames - 1));
    els.vizSub.textContent =
      `${data.frames} frames · ${(data.dt * 1000).toFixed(0)} ms/frame · ` +
      `${(data.frames * data.dt).toFixed(2)} s de ciclo`;
    els.ikWarning.classList.toggle("hidden", data.ik_ok);
    updateDerived(data);
    updateWarnings(data);
  } catch (err) {
    if (seq !== previewSeq) return;
    els.vizSub.textContent = "Error al calcular la simulación";
    toast(err.message, true);
  }
}

// ------------------------------------------------------------------ derived
function updateDerived(data) {
  const t = data.timing;
  const tstanceTitle = t.clamped
    ? `Recortado: 2L/vel pediría ${fmt(t.Tstance_raw, 3, "s")}; el límite ${fmt(t.max_stance_ratio, 1, "×")}·Tswing lo deja en ${fmt(t.Tstance, 3, "s")}`
    : "Tstance = 2L / vel (sin recorte)";
  const cells = [
    { k: "Periodo", v: fmt(t.period, 2, "s"), title: "Tstance + Tswing" },
    { k: "Tstance", v: fmt(t.Tstance, 3, "s"), warn: t.clamped, title: tstanceTitle },
    { k: "Zancada 2L", v: `${(t.stride * 1000).toFixed(0)} mm`, title: "Recorrido del pie en apoyo" },
    { k: "Pasos/s", v: fmt(t.steps_per_s, 2), title: "1 / periodo" },
    { k: "v implícita", v: fmt(t.implied_speed, 3, "m/s"), title: "Velocidad del cuerpo medida en los pies en apoyo" },
    { k: "Yaw implícito", v: fmt(t.implied_yaw_rate, 1, "°/s"), title: "Yaw del cuerpo medido en los pies en apoyo" },
    { k: "Dirección", v: fmt(t.implied_direction, 0, "°"), title: "Dirección de avance implícita (0° = adelante)" },
    { k: "IK", v: data.ik_ok ? "OK" : "FUERA", warn: !data.ik_ok, title: `${fmt(data.max_angle, 1, "°")} de ángulo máximo` },
  ];
  els.derived.innerHTML = cells
    .map(
      (c) =>
        `<div class="cell${c.warn ? " warn" : ""}" title="${escapeHtml(c.title || "")}">` +
        `<span class="k">${c.k}</span><span class="v">${escapeHtml(c.v)}</span></div>`,
    )
    .join("");
}

function updateWarnings(data) {
  const list = [...(data.warnings || [])];
  if (running) {
    list.unshift(
      "Marcha activa: los sliders solo cambian la vista previa. Pulsa Detener para aplicar otros valores.",
    );
  }
  els.warnings.classList.toggle("hidden", list.length === 0);
  els.warnings.innerHTML =
    list.length === 0
      ? ""
      : `<ul>${list.map((w) => `<li>${escapeHtml(w)}</li>`).join("")}</ul>`;
}

// ------------------------------------------------------------------ readout
function buildReadout() {
  els.readout.innerHTML = "";
  for (const leg of LEG_ORDER) {
    const div = document.createElement("div");
    div.className = "leg-readout";
    div.dataset.leg = leg;
    div.innerHTML = `
      <div class="leg-name">
        <span style="color:${LEG_COLORS[leg]}">${leg}</span>
        <span class="phase">—</span>
      </div>
      <div class="ang"><span>Roll</span><b class="a0">—</b></div>
      <div class="ang"><span>Pitch</span><b class="a1">—</b></div>
      <div class="ang"><span>Rodilla</span><b class="a2">—</b></div>
      <div class="ang"><span>Z pie</span><b class="z">—</b></div>
    `;
    div.title = LEG_NAMES[leg];
    els.readout.appendChild(div);
  }
}

function updateReadout(feet, angles, swing) {
  for (let i = 0; i < LEG_ORDER.length; i++) {
    const leg = LEG_ORDER[i];
    const div = els.readout.querySelector(`.leg-readout[data-leg="${leg}"]`);
    if (!div) continue;
    const a = angles[i];
    div.querySelector(".a0").textContent = `${a[0].toFixed(1)}°`;
    div.querySelector(".a1").textContent = `${a[1].toFixed(1)}°`;
    div.querySelector(".a2").textContent = `${a[2].toFixed(1)}°`;
    div.querySelector(".z").textContent = `${(feet[i][2] * 1000).toFixed(0)} mm`;
    const phase = div.querySelector(".phase");
    const isSwing = !!swing[i];
    phase.textContent = isSwing ? "VUELO" : "APOYO";
    phase.classList.toggle("swing", isSwing);
  }
}

function showPreviewFrame(index) {
  if (!preview) return;
  const n = preview.frames;
  const i = ((index % n) + n) % n;
  if (i === lastShown) return;
  lastShown = i;
  const d = preview.data;
  updateReadout(d.feet[i], d.angles[i], d.swing[i]);
  els.frameLabel.textContent = `${i + 1} / ${n}`;
  els.scrub.value = String(i);
}

// -------------------------------------------------------------------- canvas
function resizeCanvas() {
  const canvas = els.canvas;
  const wrap = els.canvasWrap;
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(200, wrap.clientWidth);
  const h = Math.max(200, wrap.clientHeight);
  canvas.width = Math.round(w * dpr);
  canvas.height = Math.round(h * dpr);
  canvas.style.width = `${w}px`;
  canvas.style.height = `${h}px`;
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  view.w = w;
  view.h = h;
}

const view = { w: 800, h: 500 };
const GROUND_Z = -0.174;

function baseScale() {
  return Math.min(view.w * 0.95, view.h * 1.45) * 1.15;
}

function project(point) {
  const s = baseScale() * cam.zoom;
  const cy = Math.cos(cam.yaw);
  const sy = Math.sin(cam.yaw);
  const x1 = point[0] * cy - point[1] * sy;
  const y1 = point[0] * sy + point[1] * cy;
  const cp = Math.cos(cam.pitch);
  const sp = Math.sin(cam.pitch);
  const y2 = y1 * cp - point[2] * sp;
  const z2 = y1 * sp + point[2] * cp;
  return {
    x: view.w / 2 + x1 * s,
    y: view.h * 0.56 - z2 * s,
    depth: y2,
  };
}

function rpyMatrix(rollDeg, pitchDeg, yawDeg) {
  const r = (rollDeg * Math.PI) / 180;
  const p = (pitchDeg * Math.PI) / 180;
  const y = (yawDeg * Math.PI) / 180;
  const cr = Math.cos(r), sr = Math.sin(r);
  const cp = Math.cos(p), sp = Math.sin(p);
  const cy = Math.cos(y), sy = Math.sin(y);
  // R = Rz(yaw) * Ry(pitch) * Rx(roll), igual que lib_algebra.RPY
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}

function applyR(R, v) {
  return [
    R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
    R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
    R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
  ];
}

function add(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function polygon(ctx, pts, fill, stroke, width = 1) {
  if (pts.length < 2) return;
  ctx.beginPath();
  ctx.moveTo(pts[0].x, pts[0].y);
  for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
  ctx.closePath();
  if (fill) {
    ctx.fillStyle = fill;
    ctx.fill();
  }
  if (stroke) {
    ctx.strokeStyle = stroke;
    ctx.lineWidth = width;
    ctx.stroke();
  }
}

function drawGround(ctx) {
  const z = GROUND_Z;
  const half = 0.34;
  const step = 0.05;
  ctx.save();
  ctx.strokeStyle = "rgba(42, 58, 88, 0.75)";
  ctx.lineWidth = 1;
  for (let v = -half; v <= half + 1e-9; v += step) {
    const a = project([v, -half, z]);
    const b = project([v, half, z]);
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
    const c = project([-half, v, z]);
    const d = project([half, v, z]);
    ctx.beginPath();
    ctx.moveTo(c.x, c.y);
    ctx.lineTo(d.x, d.y);
    ctx.stroke();
  }

  // Ejes del mundo sobre el suelo.
  const origin = project([0, 0, z]);
  const xEnd = project([0.16, 0, z]);
  const yEnd = project([0, 0.16, z]);
  ctx.lineWidth = 2;
  ctx.strokeStyle = "rgba(239, 68, 68, 0.8)";
  ctx.beginPath();
  ctx.moveTo(origin.x, origin.y);
  ctx.lineTo(xEnd.x, xEnd.y);
  ctx.stroke();
  ctx.strokeStyle = "rgba(34, 197, 94, 0.8)";
  ctx.beginPath();
  ctx.moveTo(origin.x, origin.y);
  ctx.lineTo(yEnd.x, yEnd.y);
  ctx.stroke();

  ctx.fillStyle = "rgba(239, 68, 68, 0.9)";
  ctx.font = "11px system-ui";
  ctx.fillText("X", xEnd.x + 4, xEnd.y + 4);
  ctx.fillStyle = "rgba(34, 197, 94, 0.9)";
  ctx.fillText("Y", yEnd.x + 4, yEnd.y + 4);
  ctx.restore();
}

function drawTrails(ctx) {
  if (!preview) return;
  const n = preview.frames;
  for (let j = 0; j < LEG_ORDER.length; j++) {
    const leg = LEG_ORDER[j];
    ctx.save();
    ctx.strokeStyle = LEG_COLORS[leg];
    ctx.globalAlpha = 0.28;
    ctx.lineWidth = 1.3;
    ctx.beginPath();
    for (let i = 0; i < n; i++) {
      const p = project(preview.data.feet[i][j]);
      if (i === 0) ctx.moveTo(p.x, p.y);
      else ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
    ctx.restore();
  }

  // Huella nominal.
  if (preview.base_feet) {
    ctx.save();
    ctx.strokeStyle = "rgba(141, 162, 192, 0.35)";
    ctx.lineWidth = 1;
    for (const leg of LEG_ORDER) {
      const b = preview.base_feet[leg];
      const p = project([b[0], b[1], GROUND_Z]);
      ctx.beginPath();
      ctx.moveTo(p.x - 4, p.y);
      ctx.lineTo(p.x + 4, p.y);
      ctx.moveTo(p.x, p.y - 4);
      ctx.lineTo(p.x, p.y + 4);
      ctx.stroke();
    }
    ctx.restore();
  }
}

function drawShadow(ctx, foot) {
  const p = project([foot[0], foot[1], GROUND_Z]);
  ctx.save();
  ctx.fillStyle = "rgba(0, 0, 0, 0.35)";
  ctx.beginPath();
  ctx.ellipse(p.x, p.y, 5.5, 2.6, 0, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function bodyToWorld(R, pos, local) {
  const r = applyR(R, local);
  return [pos[0] + r[0], pos[1] + r[1], pos[2] + r[2]];
}

/** Mezcla dos colores hex (#rrggbb); t=0 deja `a`, t=1 deja `b`. */
function mixHex(a, b, t) {
  const pa = [1, 3, 5].map((i) => parseInt(a.slice(i, i + 2), 16));
  const pb = [1, 3, 5].map((i) => parseInt(b.slice(i, i + 2), 16));
  return (
    "#" +
    pa
      .map((v, i) => Math.round(v + (pb[i] - v) * t).toString(16).padStart(2, "0"))
      .join("")
  );
}

/** Eslabón de la pierna con borde, cuerpo y brillo para dar volumen. */
function drawLink(ctx, a, b, width, color) {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const len = Math.hypot(dx, dy);
  if (len < 0.4) return;

  ctx.save();
  ctx.lineCap = "round";

  // Borde oscuro.
  ctx.strokeStyle = "rgba(5, 10, 20, 0.9)";
  ctx.lineWidth = width + Math.max(2, width * 0.5);
  ctx.beginPath();
  ctx.moveTo(a.x, a.y);
  ctx.lineTo(b.x, b.y);
  ctx.stroke();

  // Cuerpo del eslabón.
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(a.x, a.y);
  ctx.lineTo(b.x, b.y);
  ctx.stroke();

  // Brillo lateral (efecto cilindro).
  const nx = -dy / len;
  const ny = dx / len;
  const off = width * 0.26;
  ctx.strokeStyle = "rgba(255, 255, 255, 0.22)";
  ctx.lineWidth = Math.max(1, width * 0.28);
  ctx.beginPath();
  ctx.moveTo(a.x + nx * off, a.y + ny * off);
  ctx.lineTo(b.x + nx * off, b.y + ny * off);
  ctx.stroke();
  ctx.restore();
}

function drawJoint(ctx, p, radius, color) {
  ctx.save();
  ctx.beginPath();
  ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
  ctx.fillStyle = "#111c30";
  ctx.fill();
  ctx.lineWidth = Math.max(1.2, radius * 0.5);
  ctx.strokeStyle = color;
  ctx.stroke();
  ctx.restore();
}

function drawFoot(ctx, p, radius, color, swing) {
  // Suela ancha y plana: así no se confunde con una articulación.
  ctx.save();
  ctx.beginPath();
  ctx.ellipse(p.x, p.y, radius * 1.2, radius * 0.6, 0, 0, Math.PI * 2);
  if (swing) {
    ctx.fillStyle = color;
    ctx.fill();
  } else {
    ctx.fillStyle = "rgba(10, 18, 32, 0.9)";
    ctx.fill();
    ctx.strokeStyle = color;
    ctx.lineWidth = Math.max(1.6, radius * 0.34);
    ctx.stroke();
  }
  ctx.restore();
}

/** Frame del preview que toca dibujar. */
function sceneFrame() {
  if (!preview) return null;
  const n = preview.frames;
  const i = ((Math.floor(frameIdx) % n) + n) % n;
  const d = preview.data;
  return {
    feet: d.feet[i],
    swing: d.swing[i].map((s) => !!s),
    joints: d.joints ? d.joints[i] : null,
  };
}

function drawBody(ctx, body, hips, dims) {
  const R = rpyMatrix(body.rpy_deg[0], body.rpy_deg[1], body.rpy_deg[2]);
  const pos = body.pos;
  const hx = dims.hip_x / 2 + 0.045;
  const hy = dims.hip_y / 2 + 0.055;
  const top = 0.012;
  const bottom = -0.012;

  const local = [
    [-hx, -hy],
    [hx, -hy],
    [hx, hy],
    [-hx, hy],
  ];
  const topWorld = local.map(([x, y]) => add(pos, applyR(R, [x, y, top])));
  const botWorld = local.map(([x, y]) => add(pos, applyR(R, [x, y, bottom])));
  const topP = topWorld.map(project);
  const botP = botWorld.map(project);

  // Caras laterales (más oscuras).
  for (let i = 0; i < 4; i++) {
    const k = (i + 1) % 4;
    polygon(
      ctx,
      [topP[i], topP[k], botP[k], botP[i]],
      "rgba(18, 28, 46, 0.95)",
      "rgba(75, 106, 148, 0.7)",
      1,
    );
  }
  // Tapa superior.
  polygon(ctx, topP, "rgba(35, 52, 82, 0.98)", "rgba(120, 160, 214, 0.9)", 1.4);

  // Flecha de dirección (+X del cuerpo).
  const a = project(add(pos, applyR(R, [0.0, 0.0, top + 0.001])));
  const b = project(add(pos, applyR(R, [hx * 0.98, 0.0, top + 0.001])));
  ctx.save();
  ctx.strokeStyle = "#38bdf8";
  ctx.fillStyle = "#38bdf8";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(a.x, a.y);
  ctx.lineTo(b.x, b.y);
  ctx.stroke();
  const ang = Math.atan2(b.y - a.y, b.x - a.x);
  ctx.beginPath();
  ctx.moveTo(b.x, b.y);
  ctx.lineTo(b.x - 8 * Math.cos(ang - 0.4), b.y - 8 * Math.sin(ang - 0.4));
  ctx.lineTo(b.x - 8 * Math.cos(ang + 0.4), b.y - 8 * Math.sin(ang + 0.4));
  ctx.closePath();
  ctx.fill();
  ctx.restore();

  // Caderas (marcas sobre la tapa).
  ctx.save();
  ctx.fillStyle = "rgba(148, 180, 220, 0.9)";
  for (const leg of LEG_ORDER) {
    const w = add(pos, applyR(R, hips[leg]));
    const p = project(w);
    ctx.beginPath();
    ctx.arc(p.x, p.y, 2.4, 0, Math.PI * 2);
    ctx.fill();
  }
  ctx.restore();

  return { R, pos, top };
}

function drawScene() {
  const ctx = els.ctx;
  ctx.clearRect(0, 0, view.w, view.h);
  drawGround(ctx);
  drawTrails(ctx);

  if (!preview) {
    ctx.fillStyle = "rgba(141, 162, 192, 0.8)";
    ctx.font = "13px system-ui";
    ctx.textAlign = "center";
    ctx.fillText("Esperando simulación…", view.w / 2, view.h / 2);
    ctx.textAlign = "left";
    return;
  }

  const body = preview.body;
  const dims = preview.dims;
  const R = rpyMatrix(body.rpy_deg[0], body.rpy_deg[1], body.rpy_deg[2]);
  const frame = sceneFrame();
  const feet = frame ? frame.feet : LEG_ORDER.map((leg) => preview.base_feet[leg]);
  const swing = frame ? frame.swing : LEG_ORDER.map(() => false);
  const joints = frame ? frame.joints : null;

  const hipWorld = {};
  for (const leg of LEG_ORDER) hipWorld[leg] = bodyToWorld(R, body.pos, preview.hips[leg]);

  for (let j = 0; j < LEG_ORDER.length; j++) drawShadow(ctx, feet[j]);

  // El cuerpo primero: asi ninguna pata queda tapada por la tapa y las cuatro
  // posiciones se leen siempre. Es una vista esquematica, no un render solido.
  drawBody(ctx, body, preview.hips, dims);

  // Tamaños proporcionales a la escala de proyección (metros -> píxeles).
  const s = baseScale() * cam.zoom;
  const hipW = Math.max(2, 0.0072 * s);
  const thighW = Math.max(3, 0.0115 * s);
  const shinW = Math.max(2.5, 0.0098 * s);
  const hipR = Math.max(2.4, 0.0075 * s);
  const kneeR = Math.max(2.4, 0.0082 * s);
  const footR = Math.max(4, 0.0105 * s);

  // Orden de pintado por profundidad (lejos primero).
  const order = LEG_ORDER.map((leg, j) => {
    const hipP = project(hipWorld[leg]);
    const footP = project(feet[j]);
    let j1 = null;
    let j2 = null;
    if (joints && joints[leg]) {
      j1 = project(bodyToWorld(R, body.pos, joints[leg][0]));
      j2 = project(bodyToWorld(R, body.pos, joints[leg][1]));
    }
    return {
      leg,
      j,
      hipP,
      footP,
      j1,
      j2,
      depth: (hipP.depth + footP.depth) / 2,
    };
  }).sort((a, b) => b.depth - a.depth);

  for (const { leg, j, hipP, footP, j1, j2 } of order) {
    const color = LEG_COLORS[leg];
    // El eslabón L1 (brazo de cadera) va apagado; la espinilla, más clara.
    const hipLinkColor = mixHex(color, "#8da2c0", 0.55);
    const shinColor = mixHex(color, "#ffffff", 0.18);
    const isSwing = swing[j];

    // Línea punteada al suelo: ayuda a leer la altura del pie.
    const groundP = project([feet[j][0], feet[j][1], GROUND_Z]);
    ctx.save();
    ctx.setLineDash([3, 4]);
    ctx.strokeStyle = color;
    ctx.globalAlpha = isSwing ? 0.45 : 0.2;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(groundP.x, groundP.y);
    ctx.lineTo(footP.x, footP.y);
    ctx.stroke();
    ctx.restore();

    if (j1 && j2) {
      // Cadena real: cadera -> L1 -> rodilla -> L3 -> pie.
      drawLink(ctx, hipP, j1, hipW, hipLinkColor);
      drawLink(ctx, j1, j2, thighW, color);
      drawLink(ctx, j2, footP, shinW, shinColor);
      drawJoint(ctx, j1, Math.max(2, hipR * 0.8), hipLinkColor);
      drawJoint(ctx, j2, kneeR, color);
    } else {
      // Respaldo si el servidor no envia los puntos articulares.
      drawLink(ctx, hipP, footP, thighW * 0.9, color);
    }

    drawJoint(ctx, hipP, hipR, color);
    drawFoot(ctx, footP, footR, color, isSwing);

    ctx.save();
    ctx.fillStyle = color;
    ctx.font = "bold 10px system-ui";
    ctx.fillText(leg, footP.x + footR + 3, footP.y - 6);
    ctx.restore();
  }
}

// ------------------------------------------------------------------ controls
function setRunning(value) {
  running = value;
  els.runBadge.classList.toggle("hidden", !value);
  els.btnApply.disabled = value;
  els.btnStop.disabled = !value;
}

async function startGait() {
  if (!confirm("¿Ejecutar esta marcha en los servos? Asegúrate de que el robot está apoyado o levantado.")) {
    return;
  }
  try {
    const st = await api("/api/gait/start", params);
    setRunning(true);
    const baseMsg =
      st.hardware === "real"
        ? "Marcha iniciada en el robot."
        : "Marcha iniciada en simulación (sin hardware, los servos no se mueven).";
    const extra = st.warnings && st.warnings.length ? " Aviso: " + st.warnings.join(" · ") : "";
    toast(baseMsg + extra);
  } catch (err) {
    toast(err.message, true);
  }
}

async function stopGait() {
  try {
    await api("/api/gait/stop", {});
    setRunning(false);
    toast("Marcha detenida. El robot vuelve a neutro.");
  } catch (err) {
    toast(err.message, true);
  }
}

async function pollStatus() {
  try {
    const st = await api("/api/gait/status");
    setRunning(st.running);
    if (st.error && st.error !== lastRunError) {
      lastRunError = st.error;
      toast(st.error, true);
    }
    if (st.running && st.latest) {
      const latest = st.latest;
      const feet = [];
      const angles = [];
      const swing = [];
      for (const leg of LEG_ORDER) {
        feet.push(latest.feet[leg]);
        angles.push(latest.angles[leg]);
        swing.push(latest.swing[leg] ? 1 : 0);
      }
      updateReadout(feet, angles, swing);
      els.frameLabel.textContent = "en vivo";
    }
  } catch {
    /* el servidor puede estar arrancando */
  }
}

async function restPose() {
  if (running) {
    toast("Detén la marcha antes de mover los servos.", true);
    return;
  }
  if (!confirm("¿Llevar los servos a la pose neutra de marcha (IK)?")) return;
  try {
    const st = await api("/api/gait/neutral", {});
    toast(
      st.hardware === "real"
        ? "Robot en pose neutra de marcha (IK)."
        : "Simulación: pose neutra calculada (sin hardware).",
    );
  } catch (err) {
    toast(err.message, true);
  }
}

async function disableServos() {
  if (!confirm("Se liberará la fuerza de todos los servos. ¿Continuar?")) return;
  try {
    if (running) await api("/api/gait/stop", {});
    setRunning(false);
    await api("/api/disable", {});
    toast("Servos liberados.");
  } catch (err) {
    toast(err.message, true);
  }
}

function resetParams() {
  params = { ...((serverState && serverState.defaults) || FALLBACK_DEFAULTS) };
  syncControls();
  schedulePreview();
  toast("Parámetros restaurados.");
}

// -------------------------------------------------------------------- init
function bindEvents() {
  els.btnPlay.addEventListener("click", () => {
    playing = !playing;
    els.btnPlay.textContent = playing ? "Pausa" : "Reproducir";
  });

  els.scrub.addEventListener("pointerdown", () => {
    scrubbing = true;
  });
  els.scrub.addEventListener("input", () => {
    playing = false;
    els.btnPlay.textContent = "Reproducir";
    frameIdx = Number(els.scrub.value);
    showPreviewFrame(frameIdx);
    drawScene();
  });
  els.scrub.addEventListener("pointerup", () => {
    scrubbing = false;
  });

  els.speed.addEventListener("change", () => {
    speed = Number(els.speed.value) || 1;
  });

  els.btnApply.addEventListener("click", startGait);
  els.btnStop.addEventListener("click", stopGait);
  els.btnRestPose.addEventListener("click", restPose);
  els.btnDisable.addEventListener("click", disableServos);
  els.btnReset.addEventListener("click", resetParams);

  // Órbita de cámara.
  const wrap = els.canvasWrap;
  let dragging = false;
  let last = null;
  wrap.addEventListener("pointerdown", (ev) => {
    if (ev.target.closest(".canvas-hint")) return;
    dragging = true;
    last = { x: ev.clientX, y: ev.clientY };
    wrap.classList.add("dragging");
    wrap.setPointerCapture(ev.pointerId);
  });
  wrap.addEventListener("pointermove", (ev) => {
    if (!dragging || !last) return;
    cam.yaw += (ev.clientX - last.x) * 0.008;
    cam.pitch = Math.max(0.12, Math.min(1.3, cam.pitch + (ev.clientY - last.y) * 0.006));
    last = { x: ev.clientX, y: ev.clientY };
    drawScene();
  });
  const endDrag = () => {
    dragging = false;
    last = null;
    wrap.classList.remove("dragging");
  };
  wrap.addEventListener("pointerup", endDrag);
  wrap.addEventListener("pointercancel", endDrag);
  wrap.addEventListener("dblclick", () => {
    cam.yaw = -0.55;
    cam.pitch = 0.62;
    cam.zoom = 1;
    drawScene();
  });
  wrap.addEventListener(
    "wheel",
    (ev) => {
      ev.preventDefault();
      cam.zoom = Math.max(0.45, Math.min(2.6, cam.zoom * Math.exp(-ev.deltaY * 0.0012)));
      drawScene();
    },
    { passive: false },
  );

  window.addEventListener("resize", () => {
    resizeCanvas();
    drawScene();
  });

  window.addEventListener("beforeunload", () => {
    if (!running) return;
    try {
      navigator.sendBeacon(
        "/api/gait/stop",
        new Blob(["{}"], { type: "application/json" }),
      );
    } catch {
      /* nada */
    }
  });
}

function animate(ts) {
  const dt = lastRaf === null ? 0 : Math.min((ts - lastRaf) / 1000, 0.12);
  lastRaf = ts;

  if (preview && playing && !scrubbing) {
    frameIdx += dt / (preview.dt / speed);
    const n = preview.frames;
    if (frameIdx >= n) frameIdx -= n;
  }

  drawScene();
  if (!running) showPreviewFrame(Math.floor(frameIdx));
  requestAnimationFrame(animate);
}

async function init() {
  els.modeBadge = $("#mode-badge");
  els.runBadge = $("#run-badge");
  els.vizSub = $("#viz-sub");
  els.canvas = $("#scene");
  els.ctx = els.canvas.getContext("2d");
  els.canvasWrap = $("#canvas-wrap");
  els.ikWarning = $("#ik-warning");
  els.derived = $("#derived");
  els.readout = $("#readout");
  els.warnings = $("#warnings");
  els.groups = $("#groups");
  els.toast = $("#toast");
  els.btnPlay = $("#btn-play");
  els.scrub = $("#scrub");
  els.frameLabel = $("#frame-label");
  els.speed = $("#speed");
  els.btnApply = $("#btn-apply");
  els.btnStop = $("#btn-stop");
  els.btnRestPose = $("#btn-rest-pose");
  els.btnDisable = $("#btn-disable");
  els.btnReset = $("#btn-reset-params");

  buildReadout();
  bindEvents();
  resizeCanvas();

  try {
    serverState = await api("/api/gait/state");
    params = { ...serverState.defaults };
  } catch (err) {
    toast(`No se pudo leer el estado del servidor: ${err.message}`, true);
  }

  updateModeBadge();
  buildControls();
  await refreshPreview();
  setInterval(pollStatus, 700);
  requestAnimationFrame(animate);
}

init();
