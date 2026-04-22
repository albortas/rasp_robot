# import busio
# from board import SCL, SDA
# from adafruit_pca9685 import PCA9685
# from adafruit_motor import servo

class ServoControl:
    def __init__(self, file_config):
        self.fc = file_config
        self.i2c = busio.I2C(SCL, SDA)

    def _enable_pca(self):
        list_pca = []
        for servo_config in self.fc.servos.values():
            list_pca.append(servo_config["pca9685"])
        if 2 not in list_pca:
            self._pca_1 = self._create_pca(self.fc.boards["pca9685_1"])
        else:
            self._pca_1 = self._create_pca(self.fc.boards["pca9685_1"])
            self._pca_2 = self._create_pca(self.fc.boards["pca9685_2"])
                        
    def _create_pca(self, board_config):
        pca = PCA9685(self.i2c, address = int(board_config["address"]))
        pca.frequency = board_config["frequency"]
        return pca
    
    def _init_servos(self):
        servos = {}
        for name, data in self.config.servos.items():
            if data["pca9685"] == 1:
                srv = servo.Servo(self._pca_1[data["channel"]])
                srv.set_pulse_width_range(min_pulse= data["min_pulse"], max_pulse= data["max_pulse"])
                servos[name] = srv
            elif data["pca9685"] == 2:
                srv = servo.Servo(self._pca_1[data["channel"]])
                srv.set_pulse_width_range(min_pulse= data["min_pulse"], max_pulse= data["max_pulse"])
                servos[name] = srv
        return servos

    def set_servo_angle(self, name, angle_deg):
        """Envia un angulo (grados) al servo.

        Args:
            name (str): Nombre articulacion.
            logical_angle_deg (float): Angulo en grados        
        """
        # cal = self.calib_cfg[name]
        # offset = self.offsets.get(name, 0.0)
        # base = cal["zero_angle"] + logical_angle_deg + offset
        # physical = 180.0 - base if cal["invert_direction"] else base
        # physical = max(0, min(180, physical))
        self.servos[name].angle = angle_deg

    def set_leg_angles(self, leg_id, angles_deg):
        """Forma una tupla (nombre_articulacion, angulo)

        Args:
            leg_id (str): Nombres de extremidades: FL, FR, RL y RR.
            angles_deg (list): Lista de angulos de articulaciones en grados.
        """
        for name, angle in zip(self.fc.leg_map[leg_id], angles_deg, strict=True):
            self.set_servo_angle(name, angle)
    
    def deinit(self):
        self._pca_1.deinit()
        self._pca_2.deinit()

if __name__ == "__main__":
    from src.utils.config import Config
    cfg = Config()
    lista = []
    for value in cfg.servos.values():
        # print(f"-> {value["pca9685"]}")
        lista.append(value["pca9685"])
    print(lista)
    if 2 not in lista:
        print("No existe pca9685_2")
    else:
        print("Existe pca9685_2")
    