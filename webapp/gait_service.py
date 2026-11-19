"""Servicio de tuning del generador de marcha.

Envuelve RobotModel + BazierGait en un objeto de larga vida. Se usa para:

* ``preview``: simula varios ciclos de marcha y devuelve las posiciones de los
  pies y los angulos articulares de cada frame. No toca el hardware.
* ``start`` / ``stop``: ejecuta la marcha en los servos reales con un hilo a
  ``DT`` segundos por ciclo.

El servicio comparte el ``HardwareManager`` del calibrador (``ServoService``)
para que solo haya un proceso dueno del bus I2C.

Unidades de la API (pensadas para la UI):

* x, y, z ................ metros
* roll, pitch, yaw ....... grados
* L ...................... metros (semizancada; la zancada es 2*L)
* vel .................... m/s (solo fija Tstance = 2*L/vel)
* lateral ................ grados (0 = adelante, +90 = izquierda)
* yaw_rate ............... grados/s
* clearance_height ....... metros
* penetration_depth ...... metros
* Tswing ................. segundos
"""

from __future__ import annotations

import copy
import math
import threading
import time

import numpy as np

from src.gait.bezier_gait import BazierGait
from src.kinematics.robot_model import RobotModel
from src.utils.logger import log

LEG_ORDER = ("FL", "FR", "RL", "RR")
DT = 0.02  # s por frame (50 Hz, igual que main.py)

DEFAULTS = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "L": 0.025,
    "vel": 0.05,
    "lateral": 0.0,
    "yaw_rate": 0.0,
    "clearance_height": 0.04,
    "penetration_depth": 0.005,
    "Tswing": 0.25,
    "max_stance_ratio": 1.3,
}

# Limites de seguridad. El servidor recorta siempre; la UI usa los mismos.
LIMITS = {
    "x": (-0.04, 0.04),
    "y": (-0.04, 0.04),
    "z": (-0.03, 0.03),
    "roll": (-15.0, 15.0),
    "pitch": (-15.0, 15.0),
    "yaw": (-15.0, 15.0),
    "L": (0.0, 0.06),
    "vel": (0.0, 0.15),
    # LateralFraction es una direccion: +-90 cubre adelante con lateral
    # izquierda/derecha. Mas alla de 90 el pie iria hacia atras.
    "lateral": (-90.0, 90.0),
    "yaw_rate": (-30.0, 30.0),
    "clearance_height": (0.01, 0.08),
    "penetration_depth": (0.0, 0.02),
    "Tswing": (0.15, 0.4),
    "max_stance_ratio": (1.0, 5.0),
}


def _clamp(value: float, lo: float, hi: float) -> tuple[float, bool]:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"Valor no finito: {value!r}")
    if value < lo:
        return lo, True
    if value > hi:
        return hi, True
    return value, False


class GaitService:
    """Estado unico del generador de marcha + ejecucion en hardware."""

    def __init__(self, servo=None):
        self.servo = servo
        self.robot = RobotModel()
        self.base = copy.deepcopy(self.robot.WorldToFoot)
        self.lock = servo._lock if servo is not None else threading.RLock()

        self._thread = None
        self._stop = threading.Event()
        self._running = False
        self._error = None
        self._latest = None
        self._active_params = None

    # ------------------------------------------------------------------ utils
    @property
    def hw(self):
        return self.servo.hw if self.servo is not None else None

    @property
    def simulated(self) -> bool:
        return self.hw is None

    def _hardware_label(self) -> str:
        return "simulacion" if self.simulated else "real"

    def _sanitize(self, raw: dict | None) -> tuple[dict, list[str]]:
        """Recorta los parametros recibidos y devuelve avisos legibles."""
        raw = raw or {}
        params = {}
        clipped = []
        for key, default in DEFAULTS.items():
            lo, hi = LIMITS[key]
            value, was_clipped = _clamp(raw.get(key, default), lo, hi)
            params[key] = value
            if was_clipped:
                clipped.append(key)

        warnings = []
        if clipped:
            warnings.append(
                "Parametros recortados al limite: " + ", ".join(sorted(clipped))
            )
        if params["Tswing"] <= DT:
            warnings.append("Tswing debe ser mayor que el paso de simulacion.")
        if params["clearance_height"] <= params["penetration_depth"]:
            warnings.append(
                "La altura de despeje no supera la profundidad de penetracion."
            )
        if params["max_stance_ratio"] > 1.5:
            warnings.append(
                "Limite de apoyo ampliado (>1.5): el valor estable es 1.3. "
                "Probar en hardware con cuidado."
            )
        return params, warnings

    def _timing(self, p: dict) -> dict:
        """Replica el calculo de Tstance de GenerateTrajectory."""
        Tswing = p["Tswing"]
        ratio = p["max_stance_ratio"]
        if p["vel"] != 0.0:
            Tstance = 2.0 * abs(p["L"]) / abs(p["vel"])
        elif p["yaw_rate"] != 0.0:
            Tstance = ratio * Tswing
        else:
            Tstance = 0.0

        Tstance_raw = Tstance
        clamped = Tstance > ratio * Tswing
        if clamped:
            Tstance = ratio * Tswing
        if Tstance < DT:
            Tstance = 0.0

        moving = Tstance > 0.0
        period = Tstance + Tswing if moving else 0.0
        stride = 2.0 * abs(p["L"])
        # La velocidad del cuerpo la fija el pie en apoyo: recorre 2L durante
        # Tstance (el swing no cuenta para el avance del cuerpo).
        est_speed = stride / Tstance if Tstance > 0 and p["vel"] != 0.0 else 0.0
        est_yaw = math.radians(p["yaw_rate"]) if moving else 0.0

        return {
            "Tstance": round(Tstance, 4),
            "Tstance_raw": round(Tstance_raw, 4),
            "Tswing": round(Tswing, 4),
            "period": round(period, 4),
            "stride": round(stride, 4),
            "steps_per_s": round(1.0 / period, 3) if period > 0 else 0.0,
            "clamped": bool(clamped),
            "moving": bool(moving),
            "est_speed": round(est_speed, 4),
            "est_yaw_rate": round(math.degrees(est_yaw), 2),
            "max_stance_ratio": round(ratio, 3),
        }

    def _ik_ok(self, p_hf) -> bool:
        """Comprueba si la cinematica inversa tiene solucion real."""
        x, y, z = (float(v) for v in p_hf)
        L1, L2, L3 = self.robot.L1, self.robot.L2, self.robot.L3
        D = (x * x + y * y + z * z - L1 * L1 - L2 * L2 - L3 * L3) / (
            2.0 * L2 * L3
        )
        ag2 = y * y + z * z - L1 * L1
        if ag2 <= 0.0 or abs(y) < 1e-9 or not (-1.0 <= D <= 1.0):
            return False
        ag = math.sqrt(ag2)
        for side in (1.0, -1.0):
            den = 1.0 - (z / y) * (ag / (side * L1))
            if abs(den) < 1e-6:
                return False
        return True

    @staticmethod
    def _round3(values) -> list[float]:
        return [round(float(v), 5) for v in values]

    @staticmethod
    def _implied_motion(feet, swing, dt: float = DT) -> dict:
        """Velocidad lineal y yaw del cuerpo implicados por los pies en apoyo.

        Un pie en apoyo esta quieto en el mundo, asi que su movimiento en el
        marco del cuerpo es el opuesto al del cuerpo. Se toma la mediana sobre
        todos los pares de frames en apoyo para descartar los bordes.
        """
        vx_samples = []
        vy_samples = []
        w_samples = []
        for k in range(1, len(feet)):
            for j in range(4):
                if swing[k][j] or swing[k - 1][j]:
                    continue
                x0, y0 = feet[k - 1][j][:2]
                x1, y1 = feet[k][j][:2]
                vx = -(x1 - x0) / dt
                vy = -(y1 - y0) / dt
                r2 = x0 * x0 + y0 * y0
                if r2 < 1e-8:
                    continue
                # Velocidad angular del pie; el cuerpo gira al contrario.
                w = -((x0 * (y1 - y0) - y0 * (x1 - x0)) / dt) / r2
                vx_samples.append(vx)
                vy_samples.append(vy)
                w_samples.append(w)

        if not vx_samples:
            return {
                "implied_speed": 0.0,
                "implied_yaw_rate": 0.0,
                "implied_direction": 0.0,
            }

        vx = float(np.median(vx_samples))
        vy = float(np.median(vy_samples))
        w = math.degrees(float(np.median(w_samples)))
        speed = math.hypot(vx, vy)
        # El redondeo de las posiciones deja ruido sub-grado; no mostrarlo.
        if abs(w) < 0.5:
            w = 0.0
        if speed < 0.001:
            speed = 0.0
        return {
            "implied_speed": round(speed, 4),
            "implied_yaw_rate": round(w, 2),
            "implied_direction": round(math.degrees(math.atan2(vy, vx)), 1)
            if speed > 0.0
            else 0.0,
        }

    def _body(self, p: dict) -> tuple[np.ndarray, np.ndarray]:
        rpy = np.radians([p["roll"], p["pitch"], p["yaw"]])
        pos = np.array([p["x"], p["y"], p["z"]], dtype=float)
        return rpy, pos

    def _solve(self, rpy, pos, T_bf):
        """HipToFoot + IK reutilizando el mismo vector cadera-pie.

        RobotModel.IK vuelve a llamar a HipToFoot, asi que resolver las cuatro
        patas directamente evita duplicar ese trabajo en cada frame.
        """
        p_hf = self.robot.HipToFoot(rpy, pos, T_bf)
        angles = np.zeros((4, 3))
        for j, leg in enumerate(LEG_ORDER):
            angles[j] = np.degrees(
                self.robot.inverse.solve(self.robot.Legs[leg], p_hf[leg])
            )
        return p_hf, angles

    # ----------------------------------------------------------------- estado
    def state(self) -> dict:
        return {
            "hardware": self._hardware_label(),
            "error": getattr(self.servo, "hw_error", None) if self.servo else None,
            "running": self._running,
            "dt": DT,
            "defaults": DEFAULTS,
            "limits": LIMITS,
            "legs": list(LEG_ORDER),
        }

    def status(self) -> dict:
        with self.lock:
            return {
                "hardware": self._hardware_label(),
                "running": self._running,
                "error": self._error,
                "params": self._active_params,
                "latest": self._latest,
            }

    # --------------------------------------------------------------- preview
    def preview(self, raw: dict | None) -> dict:
        """Simula la marcha y devuelve frames para animar en el navegador."""
        p, warnings = self._sanitize(raw)
        timing = self._timing(p)
        rpy, pos = self._body(p)

        gen = BazierGait(
            Tswing=p["Tswing"],
            dt=DT,
            max_stance_ratio=p["max_stance_ratio"],
        )
        yaw_rad = math.radians(p["yaw_rate"])
        lateral_rad = math.radians(p["lateral"])

        feet = []
        angles = []
        swing = []
        bad_frames = 0
        max_angle = 0.0
        warmup = 0

        def step():
            return gen.GenerateTrajectory(
                L=p["L"],
                LateralFraction=lateral_rad,
                YawRate=yaw_rad,
                vel=p["vel"],
                T_bf_=self.base,
                T_bf_curr=self.base,
                clearance_height=p["clearance_height"],
                penetration_depth=p["penetration_depth"],
                contacts=[1, 1, 1, 1],
                dt=DT,
            )

        def capture(T_bf):
            nonlocal bad_frames, max_angle
            frame_feet = []
            frame_angles = []
            frame_swing = []
            frame_ok = True

            # HipToFoot devuelve el vector cadera-pie en el marco del cuerpo,
            # que es justo lo que usa IK. Lo calculamos una vez por pata.
            p_hf, ik = self._solve(rpy, pos, T_bf)

            for j, leg in enumerate(LEG_ORDER):
                xyz = T_bf[leg][:3, 3]
                frame_feet.append(self._round3(xyz))
                frame_angles.append([round(float(v), 2) for v in ik[j]])
                frame_swing.append(1 if gen.Phases[j] >= 1.0 else 0)
                max_angle = max(max_angle, float(np.max(np.abs(ik[j]))))
                if not self._ik_ok(p_hf[leg]):
                    frame_ok = False

            if not frame_ok:
                bad_frames += 1

            feet.append(frame_feet)
            angles.append(frame_angles)
            swing.append(frame_swing)

        if timing["moving"]:
            # Calentar dos periodos y capturar exactamente un ciclo: desde un
            # cruce de fase de FL hasta el siguiente. Asi el bucle de la
            # animacion empalma sin salto y el payload es pequeno.
            period = timing["period"]
            warmup = max(0, int(round(2.0 * period / DT)))
            max_steps = warmup + max(200, int(round(4.0 * period / DT)))
            phase_prev = gen.Phases[0]
            started = False

            for i in range(max_steps):
                T_bf = step()
                phase = gen.Phases[0]
                wrapped = phase < phase_prev - 0.5
                phase_prev = phase

                if i < warmup:
                    continue
                if not started:
                    if not wrapped:
                        continue
                    started = True
                elif wrapped and len(feet) > 2:
                    # El frame actual ya es el inicio del ciclo siguiente.
                    break

                capture(T_bf)
        else:
            warnings.append(
                "Sin marcha: hace falta vel>0 con L>0, o yaw_rate distinto de 0."
            )
            for _ in range(2):
                capture(step())

        # Velocidad y yaw reales implicados por el movimiento de los pies en
        # apoyo (mejor que estimarlos solo con la formula).
        timing.update(self._implied_motion(feet, swing))

        if bad_frames:
            warnings.append(
                f"IK sin solucion en {bad_frames}/{len(feet)} frames: "
                "reduce x/y/z, L o el angulo de Euler."
            )
        if max_angle > 120.0:
            warnings.append(
                f"Angulo articular maximo {max_angle:.0f} grados (>120): "
                "la postura esta cerca del limite."
            )
        if timing["clamped"]:
            warnings.append(
                f"Tstance recortado: 2L/vel = {timing['Tstance_raw']:.3f} s "
                f"supera el limite {timing['max_stance_ratio']:.1f}*Tswing = "
                f"{timing['Tstance']:.3f} s. Sube 'Limite Tstance/Tswing' para "
                "que Tstance varie con vel (experimental)."
            )

        body = {
            "pos": self._round3(pos),
            "rpy_deg": [round(float(v), 2) for v in (p["roll"], p["pitch"], p["yaw"])],
        }
        hips = {
            leg: self._round3(self.robot.WorldToHip[leg][:3, 3])
            for leg in LEG_ORDER
        }

        return {
            "hardware": self._hardware_label(),
            "dt": DT,
            "frames": len(feet),
            "warmup": warmup,
            "params": p,
            "timing": timing,
            "warnings": warnings,
            "body": body,
            "hips": hips,
            "base_feet": {
                leg: self._round3(self.base[leg][:3, 3]) for leg in LEG_ORDER
            },
            "dims": {
                "hip_x": self.robot.hip_x,
                "hip_y": self.robot.hip_y,
                "height": self.robot.height,
            },
            "ik_ok": bad_frames == 0,
            "max_angle": round(max_angle, 1),
            "data": {"feet": feet, "angles": angles, "swing": swing},
        }

    # ---------------------------------------------------------------- ejecutar
    def _send(self, angles_deg) -> None:
        if self.hw is None:
            return
        for i, leg in enumerate(LEG_ORDER):
            self.hw.set_leg_angles(leg, angles_deg[i])

    def _send_neutral(self) -> None:
        if self.hw is None:
            return
        self._send(
            np.degrees(
                self.robot.IK(
                    np.zeros(3), np.zeros(3), self.base
                )
            )
        )

    def neutral(self) -> dict:
        """Lleva los servos a la pose neutra de la marcha (IK con rpy=pos=0).

        Es distinta de la pose de calibracion (rest_angle) que usa la pagina
        de calibracion: aqui el robot queda en la postura de pie neutra.
        """
        with self.lock:
            if self._running:
                raise ValueError("Deten la marcha antes de mover a neutro.")
            self._send_neutral()
        return self.status()

    def start(self, raw: dict | None) -> dict:
        p, warnings = self._sanitize(raw)

        # Validar el ciclo completo antes de mover ningun servo: si la IK no
        # tiene solucion, es preferible no arrancar.
        check = self.preview(p)
        if not check["ik_ok"]:
            raise ValueError(
                "IK sin solucion para esta postura/marcha. Reduce x/y/z, L o "
                "los angulos de Euler antes de ejecutar."
            )
        p = check["params"]
        timing = check["timing"]
        warnings = list(dict.fromkeys(warnings + check["warnings"]))
        if not timing["moving"]:
            warnings.append(
                "Sin marcha: los servos se quedaran en la postura de la simulacion."
            )

        with self.lock:
            if self._running:
                raise ValueError("Ya hay una marcha activa. Pulsa Detener primero.")
            self._stop.clear()
            self._error = None
            self._running = True
            self._active_params = p
            # Punto de partida conocido y suave.
            self._send_neutral()
            self._thread = threading.Thread(
                target=self._loop,
                args=(p,),
                name="gait-run",
                daemon=True,
            )
            self._thread.start()
        result = self.status()
        result["warnings"] = warnings
        return result

    def stop(self) -> dict:
        self._stop.set()
        thread = self._thread
        was_running = self._running or thread is not None
        if thread is not None and thread.is_alive():
            # No retener el lock mientras el hilo termina: el hilo lo necesita
            # para enviar el ultimo frame y volver a neutro.
            thread.join(timeout=2.0)
        with self.lock:
            self._running = False
            self._thread = None
            self._latest = None
            if was_running:
                try:
                    self._send_neutral()
                except Exception as exc:  # noqa: BLE001
                    self._error = f"No se pudo volver a neutro: {exc}"
                    log.error(self._error)
        return self.status()

    def _loop(self, p: dict) -> None:
        gen = BazierGait(
            Tswing=p["Tswing"],
            dt=DT,
            max_stance_ratio=p["max_stance_ratio"],
        )
        rpy, pos = self._body(p)
        yaw_rad = math.radians(p["yaw_rate"])
        lateral_rad = math.radians(p["lateral"])

        # Transicion suave desde la pose neutra durante los primeros 0.5 s.
        neutral_deg = np.degrees(
            self.robot.IK(np.zeros(3), np.zeros(3), self.base)
        )
        start_t = time.perf_counter()
        next_t = time.perf_counter()
        try:
            while not self._stop.is_set():
                with self.lock:
                    T_bf = gen.GenerateTrajectory(
                        L=p["L"],
                        LateralFraction=lateral_rad,
                        YawRate=yaw_rad,
                        vel=p["vel"],
                        T_bf_=self.base,
                        T_bf_curr=self.base,
                        clearance_height=p["clearance_height"],
                        penetration_depth=p["penetration_depth"],
                        contacts=[1, 1, 1, 1],
                        dt=DT,
                    )
                    _, ik = self._solve(rpy, pos, T_bf)
                    self._latest = {
                        "feet": {
                            leg: self._round3(T_bf[leg][:3, 3])
                            for leg in LEG_ORDER
                        },
                        "angles": {
                            leg: [round(float(v), 2) for v in ik[i]]
                            for i, leg in enumerate(LEG_ORDER)
                        },
                        "swing": {
                            leg: bool(gen.Phases[i] >= 1.0)
                            for i, leg in enumerate(LEG_ORDER)
                        },
                    }
                    # Mezcla lineal neutro -> marcha al arrancar.
                    alpha = min(1.0, (time.perf_counter() - start_t) / 0.5)
                    self._send(neutral_deg + alpha * (ik - neutral_deg))

                next_t += DT
                delay = next_t - time.perf_counter()
                if delay > 0:
                    self._stop.wait(delay)
                else:
                    next_t = time.perf_counter()
        except Exception as exc:  # noqa: BLE001
            self._error = str(exc)
            log.error(f"Error en marcha: {exc}")
        finally:
            try:
                with self.lock:
                    self._send_neutral()
            except Exception as exc:  # noqa: BLE001
                log.error(f"No se pudo volver a neutro: {exc}")
            with self.lock:
                self._running = False
                self._latest = None
