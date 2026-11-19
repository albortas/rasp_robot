const SPAN_DEFAULT = 60;

let state = null;
let dirty = false;
const pendingAngle = new Map();

// ------------------------------------------------------------------ helpers
async function api(path, body) {
  const res = await fetch(path, {
    method: body === undefined ? "GET" : "POST",
    headers: body === undefined ? undefined : { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`);
  return data;
}

const $ = (sel) => document.querySelector(sel);

function toast(message, isError = false) {
  const el = $("#toast");
  el.textContent = message;
  el.classList.toggle("error", isError);
  el.classList.remove("hidden");
  clearTimeout(toast._t);
  toast._t = setTimeout(() => el.classList.add("hidden"), 2600);
}

function setDirty(value) {
  dirty = value;
  $("#dirty").classList.toggle("hidden", !dirty);
}

function servoByName(name) {
  return state ? state.servos.find((s) => s.name === name) : null;
}

// ------------------------------------------------------------------- estado
function updateStatus() {
  const badge = $("#mode-badge");
  if (!state) {
    badge.textContent = "sin conexión";
    badge.className = "badge sim";
    return;
  }
  const real = state.hardware === "real";
  badge.textContent = real ? "MODO REAL" : "MODO SIMULACIÓN";
  badge.className = `badge ${real ? "real" : "sim"}`;
  badge.title = state.error || "";
  $("#pcas-info").textContent = `PCA9685: ${state.num_pcas}`;
}

function applyState(data, { resetSliders = true } = {}) {
  const previous = state ? state.servos.length : -1;
  state = data;
  const rendered = document.querySelectorAll(".card").length;
  if (rendered !== state.servos.length) {
    render();
    updateStatus();
    return;
  }
  for (const servo of state.servos) {
    const card = document.querySelector(`.card[data-name="${servo.name}"]`);
    if (card) updateCard(card, servo, resetSliders);
  }
  updateStatus();
}

// -------------------------------------------------------------------- render
function render() {
  const container = $("#legs");
  container.innerHTML = "";
  for (const leg of state.legs) {
    const servos = state.servos.filter((s) => s.leg === leg);
    if (!servos.length) continue;
    const section = document.createElement("section");
    section.className = "leg";
    const h2 = document.createElement("h2");
    h2.textContent = leg;
    section.appendChild(h2);
    for (const servo of servos) section.appendChild(createCard(servo));
    container.appendChild(section);
  }
}

function createCard(servo) {
  const card = document.createElement("article");
  card.className = "card";
  card.dataset.name = servo.name;

  card.innerHTML = `
    <header class="card-head">
      <span class="joint">${servo.joint}</span>
      <span class="channel">PCA ${servo.pca9685} · ch ${servo.channel}</span>
    </header>
    <div class="row">
      <span class="label">Offset</span>
      <span class="offset value">${servo.offset}</span>
      <div class="btns">
        <button type="button" data-act="offset" data-delta="-1">−</button>
        <button type="button" data-act="offset" data-delta="1">+</button>
      </div>
    </div>
    <div class="row">
      <button type="button" class="toggle" data-act="invert"></button>
    </div>
    <div class="slider-block">
      <input type="range" step="1" data-act="angle">
      <output class="angle-out"></output>
    </div>
    <div class="row presets">
      <button type="button" data-act="preset" data-delta="-45">−45°</button>
      <button type="button" data-act="preset" data-delta="0">Reposo</button>
      <button type="button" data-act="preset" data-delta="45">+45°</button>
    </div>
  `;

  card.addEventListener("click", async (event) => {
    const btn = event.target.closest("button[data-act]");
    if (!btn) return;
    const act = btn.dataset.act;
    try {
      if (act === "offset") {
        const delta = Number(btn.dataset.delta);
        applyState(await api(`/api/servo/${servo.name}/offset`, { delta }));
        setDirty(true);
      } else if (act === "invert") {
        applyState(await api(`/api/servo/${servo.name}/invert`, {}));
        setDirty(true);
      } else if (act === "preset") {
        const delta = Number(btn.dataset.delta);
        const current = servoByName(servo.name);
        const target = current.rest_angle + delta;
        state = await api(`/api/servo/${servo.name}/angle`, { angle: target });
        setSlider(card, target);
      }
    } catch (err) {
      toast(err.message, true);
    }
  });

  const slider = card.querySelector('input[type="range"]');
  slider.addEventListener("input", () => {
    card.querySelector(".angle-out").textContent = `${slider.value}°`;
    scheduleAngle(servo.name, Number(slider.value));
  });

  updateCard(card, servo);
  return card;
}

function setSlider(card, value) {
  const slider = card.querySelector('input[type="range"]');
  if (!slider) return;
  const clamped = Math.min(Math.max(value, Number(slider.min)), Number(slider.max));
  slider.value = clamped;
  card.querySelector(".angle-out").textContent = `${clamped}°`;
}

function updateCard(card, servo, resetSlider = true) {
  card.querySelector(".offset").textContent = servo.offset;

  const toggle = card.querySelector(".toggle");
  toggle.textContent = `Inversion: ${servo.invert_direction ? "ON" : "OFF"}`;
  toggle.classList.toggle("active", servo.invert_direction);

  const span = (state && state.preview_span) || SPAN_DEFAULT;
  const slider = card.querySelector('input[type="range"]');
  slider.min = servo.rest_angle - span;
  slider.max = servo.rest_angle + span;
  if (resetSlider) setSlider(card, servo.rest_angle);
}

// ------------------------------------------------------------------ acciones
function scheduleAngle(name, angle) {
  if (pendingAngle.has(name)) clearTimeout(pendingAngle.get(name));
  pendingAngle.set(
    name,
    setTimeout(async () => {
      pendingAngle.delete(name);
      try {
        state = await api(`/api/servo/${name}/angle`, { angle });
      } catch (err) {
        toast(err.message, true);
      }
    }, 70),
  );
}

async function save() {
  try {
    applyState(await api("/api/save", {}), { resetSliders: false });
    setDirty(false);
    toast("Configuracion guardada en robot.toml");
  } catch (err) {
    toast(err.message, true);
  }
}

async function restPose() {
  try {
    applyState(await api("/api/rest-pose", {}));
    toast("Todos los servos a posicion de reposo");
  } catch (err) {
    toast(err.message, true);
  }
}

async function disableAll() {
  if (!confirm("Se liberara la fuerza de todos los servos. ¿Continuar?")) return;
  try {
    applyState(await api("/api/disable", {}), { resetSliders: false });
    toast("Servos liberados (duty_cycle = 0)");
  } catch (err) {
    toast(err.message, true);
  }
}

// --------------------------------------------------------------------- init
async function init() {
  try {
    applyState(await api("/api/state"));
  } catch (err) {
    toast(`No se pudo conectar con el servidor: ${err.message}`, true);
    updateStatus();
  }
}

$("#btn-save").addEventListener("click", save);
$("#btn-rest-pose").addEventListener("click", restPose);
$("#btn-disable").addEventListener("click", disableAll);

document.addEventListener("keydown", (event) => {
  if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "s") {
    event.preventDefault();
    save();
  }
});

window.addEventListener("beforeunload", (event) => {
  if (!dirty) return;
  event.preventDefault();
  event.returnValue = "";
});

init();
