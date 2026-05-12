from busio import I2C
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.utils.servo_repository import ServoRepository
    from src.utils.toml_loader import TomlLoader

class HardwareManager:
    """
    Administrador principal de hardware para los servos del Robodelta.
    Se integra con ServoRepository y TomlLoader para manejar placas PCA9685 de manera dinamica.
    """
    def __init__(self, loader: 'TomlLoader', repository: 'ServoRepository'):
        self.loader = loader
        self.repository = repository
        self.i2c = I2C(SCL, SDA)
        self.pcas = {}
        self._init_boards()
        self.go_to_rest_pose()

    def _init_boards(self):
        """Inicializa las placas PCA9685 necesarias segun la configuracion de los servos."""
        boards_config = self.loader.boards
        
        # Encontrar que placas PCA necesitan ser inicializadas
        needed_pcas = set()
        for servo in self.repository.get_servos():
            needed_pcas.add(servo.pca9685)
            
        if 1 in needed_pcas and "pca9685_1" in boards_config:
            b1_cfg = boards_config["pca9685_1"]
            # Convertir address a int de forma segura (ej. "0x40" -> 64)
            address_1 = int(str(b1_cfg["address"]), 0) if isinstance(b1_cfg["address"], str) else b1_cfg["address"]
            self.pcas[1] = PCA9685(self.i2c, address=address_1)
            self.pcas[1].frequency = b1_cfg["frequency"]
            
        if 2 in needed_pcas and "pca9685_2" in boards_config:
            b2_cfg = boards_config["pca9685_2"]
            address_2 = int(str(b2_cfg["address"]), 0) if isinstance(b2_cfg["address"], str) else b2_cfg["address"]
            self.pcas[2] = PCA9685(self.i2c, address=address_2)
            self.pcas[2].frequency = b2_cfg["frequency"]

    def set_angle(self, name: str, angle: float):
        """
        Envia un angulo (grados) al servo fisico especificado por su nombre.
        Calcula el duty cycle manualmente basandose en min_pulse, max_pulse, offset e invert_direction.
        """
        servo = self.repository.select_servo(name)
        
        # 1. Aplicar inversion de direccion (asumimos un rango base de 0 a 180)
        if servo.invert_direction:
            angle = 180.0 - angle
            
        # 2. Aplicar offset de calibracion
        angle += servo.offset
        
        # 3. Limitar angulo entre 0 y 180 grados de forma segura
        angle = max(0.0, min(180.0, angle))
        
        # 4. Mapear a rango de pulso (min_pulse a max_pulse)
        pulse_range = servo.max_pulse - servo.min_pulse
        pulse_us = servo.min_pulse + (pulse_range * angle / 180.0)
        
        # 5. Enviar a la placa PCA
        if servo.pca9685 in self.pcas:
            pca = self.pcas[servo.pca9685]
            pulse_seconds = pulse_us / 1_000_000.0
            
            # Calcular el duty cycle (0 a 0xFFFF, o sea 65535)
            duty_cycle = int(pulse_seconds * pca.frequency * 0xFFFF)
            
            # Asignar al canal correspondiente
            pca.channels[servo.channel].duty_cycle = duty_cycle
        else:
            raise RuntimeError(f"PCA9685 ID {servo.pca9685} no esta inicializada para el servo {name}")

    def set_leg_angles(self, leg_id: str, angles_deg: list):
        """
        Envio en lote de angulos a una pata especifica.
        Args:
            leg_id (str): ID de la pata (ej. 'FL', 'FR', 'RL', 'RR')
            angles_deg (list): Lista de angulos correspondientes a los servos de la pata
        """
        leg_servos = self.repository.get_by_leg(leg_id)
        if len(leg_servos) != len(angles_deg):
            raise ValueError(f"La pata {leg_id} tiene {len(leg_servos)} servos, pero se pasaron {len(angles_deg)} angulos.")
            
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
        print(f"No se pudo inicializar el hardware I2C (esto es normal si no estas en la Raspberry Pi):\n{e}")
