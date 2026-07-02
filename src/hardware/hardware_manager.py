from dataclasses import dataclass
from busio import I2C
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from src.utils.logger import log
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.utils.servo_repository import ServoRepository
    from src.utils.toml_loader import TomlLoader

@dataclass(frozen=True)
class CalibrationParams:
    """
    Parametros de calibracion para los angulos de reposo.
    Inmutables para evitar cambios accidentales durante la calibracion.
    Se pueden ajustar segun la resolucion cinematica inversa del robot.
    """
    hip_zero_angle: int = 90
    knee_zero_angle: int = 180

class HardwareManager:
    """
    Administrador principal de hardware para los servos del Robodelta.
    Se integra con ServoRepository y TomlLoader para manejar placas PCA9685 de manera dinamica.
    """

    def __init__(self, loader: "TomlLoader", repository: "ServoRepository"):
        self.loader = loader
        self.repository = repository
        self.config = CalibrationParams()
        self.i2c = I2C(SCL, SDA)
        self.pcas = {}
        self._init_boards()
        self.go_to_rest_pose()

    def _init_boards(self):
        """Inicializa placas PCA9685 dinamicamente segun configuracion."""
        boards_config = self.loader.boards

        # Encontrar que placas PCA necesitan ser inicializadas
        needed_pcas = {servo.pca9685 for servo in self.repository.get_servos()}
        log.info(f"Inicializando placas PCA: {needed_pcas}")

        for pca_id in needed_pcas:
            config_key = f"pca9685_{pca_id}"
            if config_key not in boards_config:
                raise RuntimeError(f"Configuracion para {config_key} no encontrada")

            cfg = boards_config[config_key]
            self.pcas[pca_id] = PCA9685(self.i2c, address=cfg["address"])
            self.pcas[pca_id].frequency = cfg["frequency"]
            log.debug(
                f"PCA {pca_id}: addr=0x{cfg['address']:02X}, freq={cfg['frequency']}Hz"
            )

    def set_angle(self, name: str, angle: float):
        """
        Envia un angulo (grados) al servo fisico especificado por su nombre.
        Calcula el duty cycle manualmente basandose en min_pulse, max_pulse, offset e invert_direction.
        """
        servo = self.repository.select_servo(name)

        # 1. Aplicar angulo cero de calibracion segun resolucion cinematica inversa.
        calib = self.config.hip_zero_angle if "hip" in name.split("_") else self.config.knee_zero_angle
        print(f"Calibracion: {name}, {calib}")
        
        # 2. Aplicar offset de calibracion
        offset = servo.offset
        base = calib + angle + offset
        
        # 3. Aplicar inversion de direccion (asumimos un rango base de 0 a 180)
        angle = 180.0 - base if servo.invert_direction else base

        # 4. Limitar angulo entre 0 y 180 grados de forma segura
        angle = max(0.0, min(180.0, angle))
        # print(f"limitar angulo: {name}, {angle}")

        # 5. Mapear a rango de pulso (min_pulse a max_pulse)
        pulse_range = servo.max_pulse - servo.min_pulse
        pulse_us = servo.min_pulse + (pulse_range * angle / 180.0)

        # 6. Enviar a la placa PCA
        if servo.pca9685 in self.pcas:
            pca = self.pcas[servo.pca9685]
            pulse_seconds = pulse_us / 1_000_000

            # Calcular el duty cycle (0 a 0xFFFF, o sea 65535)
            duty_cycle = int(pulse_seconds * pca.frequency * 0xFFFF)

            # Asignar al canal correspondiente
            pca.channels[servo.channel].duty_cycle = duty_cycle
        else:
            raise RuntimeError(
                f"PCA9685 ID {servo.pca9685} no esta inicializada para el servo {name}"
            )

    def set_leg_angles(self, leg_id: str, angles_deg: list):
        """
        Envio en lote de angulos a una pata especifica.
        Args:
            leg_id (str): ID de la pata (ej. 'FL', 'FR', 'RL', 'RR')
            angles_deg (list): Lista de angulos correspondientes a los servos de la pata
        """
        leg_servos = self.repository.get_by_leg(leg_id)
        if len(leg_servos) != len(angles_deg):
            raise ValueError(
                f"La pata {leg_id} tiene {len(leg_servos)} servos, pero se pasaron {len(angles_deg)} angulos."
            )

        for servo, angle in zip(leg_servos, angles_deg):
            self.set_angle(servo.name, angle)

    def go_to_rest_pose(self):
        """Mueve todos los servos a su 'rest_angle' (posicion de reposo)."""
        for servo in self.repository.get_servos():
            # Solo intentamos mover si su placa esta inicializada
            if servo.pca9685 in self.pcas:
                self.set_angle(servo.name, servo.rest_angle)

    def disable_all(self):
        """Apaga todos los canales mandando duty_cycle a 0, liberando la fuerza de los servos."""
        for pca in self.pcas.values():
            for channel in range(16):
                pca.channels[channel].duty_cycle = 0

    def deinit(self):
        """Libera los recursos de las placas I2C y apaga los servos."""
        self.disable_all()
        for pca in self.pcas.values():
            pca.deinit()


if __name__ == "__main__":
    from src.utils.servo_repository import ServoRepository
    from src.utils.toml_loader import TomlLoader

    repo = ServoRepository()
    loader = TomlLoader(repo)
    loader.load_from_file()

    try:
        hw = HardwareManager(loader, repo)
        print("Hardware inicializado correctamente.")
        print(f"Placas PCA detectadas e inicializadas: {list(hw.pcas.keys())}")

        # Ejemplo: hw.set_angle("FL_hip_roll", 90)
        hw.deinit()
        print("Recursos liberados correctamente.")
    except Exception as e:
        print(
            f"No se pudo inicializar el hardware I2C (esto es normal si no estas en la Raspberry Pi):\n{e}"
        )
