"""Logica compartida del calibrador web.

Envuelve ServoRepository, TomlLoader y HardwareManager en un unico objeto de
larga vida. La API Flask solo llama a estos metodos: nunca debe crearse un
HardwareManager por peticion (reabriria el bus I2C y moveria los servos).
"""

import threading

from src.utils.channels_configurator import ChannelsConfigurator
from src.utils.logger import log
from src.utils.servo_repository import ServoRepository
from src.utils.toml_loader import TomlLoader

try:
    from src.hardware.hardware_manager import HardwareManager

    HARDWARE_AVAILABLE = True
    IMPORT_ERROR = ""
except Exception as exc:  # noqa: BLE001 - en un PC sin librerias es lo esperado
    HARDWARE_AVAILABLE = False
    IMPORT_ERROR = str(exc)

LEG_ORDER = ("FL", "FR", "RL", "RR")
PREVIEW_SPAN = 60  # grados a cada lado del reposo en los sliders de la UI


def _joint_label(name: str) -> str:
    """'FL_hip_roll' -> 'Hip Roll'."""
    joint = name.split("_", 1)[1] if "_" in name else name
    return joint.replace("_", " ").title()


class ServoService:
    """Estado unico de calibracion + acceso serializado al hardware."""

    def __init__(self, num_pcas: int = 1):
        self.num_pcas = num_pcas
        self._lock = threading.RLock()

        self.repo = ServoRepository()
        self.loader = TomlLoader(self.repo)
        self.configurator = ChannelsConfigurator(self.repo)

        # Igual que en calibrate_servo.py: reasignar canales ANTES de abrir
        # el hardware, para que solo se inicialicen las placas necesarias.
        self.configurator.update_channels(num_pcas)

        self.hw = None
        self.hw_error = IMPORT_ERROR
        if HARDWARE_AVAILABLE:
            try:
                self.hw = HardwareManager(self.loader, self.repo)
            except Exception as exc:  # noqa: BLE001
                self.hw_error = str(exc)
                log.error(f"Error inicializando Hardware: {exc}")

        if self.hw is None:
            log.warning(f"Calibrador web en modo simulacion: {self.hw_error}")
        else:
            log.info("Calibrador web con hardware real")

    # ------------------------------------------------------------------ utils
    @property
    def simulated(self) -> bool:
        return self.hw is None

    def _servo(self, name: str):
        return self.repo.select_servo(name)

    def _payload(self, name: str) -> dict:
        servo = self._servo(name)
        return {
            "name": servo.name,
            "leg": servo.name[:2],
            "joint": _joint_label(servo.name),
            "pca9685": servo.pca9685,
            "channel": servo.channel,
            "min_pulse": servo.min_pulse,
            "max_pulse": servo.max_pulse,
            "rest_angle": servo.rest_angle,
            "offset": servo.offset,
            "invert_direction": servo.invert_direction,
        }

    def _send(self, name: str, angle: float) -> None:
        if self.hw is not None:
            self.hw.set_angle(name, float(angle))

    # ------------------------------------------------------------------ state
    def state(self) -> dict:
        with self._lock:
            servos = [self._payload(name) for name in self.repo.get_servo_names()]
        return {
            "hardware": "simulacion" if self.simulated else "real",
            "error": self.hw_error or None,
            "num_pcas": self.num_pcas,
            "preview_span": PREVIEW_SPAN,
            "legs": list(LEG_ORDER),
            "servos": servos,
        }

    # --------------------------------------------------------------- acciones
    def set_angle(self, name: str, angle: float) -> dict:
        with self._lock:
            self._servo(name)  # valida que existe
            self._send(name, angle)
            return self.state()

    def change_offset(self, name: str, delta: int) -> dict:
        with self._lock:
            servo = self._servo(name)
            self.repo.update_servo(name, offset=servo.offset + int(delta))
            # Reenviar el reposo para ver el efecto del offset al instante.
            self._send(name, servo.rest_angle)
            return self.state()

    def set_offset(self, name: str, value: int) -> dict:
        with self._lock:
            servo = self._servo(name)
            self.repo.update_servo(name, offset=int(value))
            self._send(name, servo.rest_angle)
            return self.state()

    def toggle_invert(self, name: str) -> dict:
        with self._lock:
            servo = self._servo(name)
            self.repo.update_servo(name, invert_direction=not servo.invert_direction)
            self._send(name, servo.rest_angle)
            return self.state()

    def test_preset(self, name: str, delta: int = 0) -> dict:
        with self._lock:
            servo = self._servo(name)
            self._send(name, servo.rest_angle + int(delta))
            return self.state()

    def rest_pose(self) -> dict:
        with self._lock:
            for servo in self.repo.get_servos():
                self._send(servo.name, servo.rest_angle)
            return self.state()

    def disable(self) -> dict:
        """Libera la fuerza de todos los servos sin cerrar el bus I2C."""
        with self._lock:
            if self.hw is not None:
                self.hw.disable_all()
            return self.state()

    def save(self) -> dict:
        with self._lock:
            self.loader.synchronize()
            data = self.state()
        data["saved"] = True
        return data
