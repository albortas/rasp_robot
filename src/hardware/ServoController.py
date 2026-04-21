import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class ServoController:
    def __init__(self, file_config):
        self.config = file_config
        self._init_pca9685()
        self._init_servos()
    
    def _init_pca9685(self):
        board_1 = self.config.boards["pca9685_1"]
        board_2 = self.config.boards["pca9685_2"]
        
        # Inicial comunicacion I2C
        i2c = busio.I2C(SCL, SDA)

        # Crear controlador PCA9685
        self.pca_1 = PCA9685(i2c, address=int(board_1["address"]))
        self.pca_2 = PCA9685(i2c, address=int(board_2["address"]))
        self.pca_1.frequency = board_1["frequency"]
        self.pca_2.frequency = board_2["frequency"]
    
    def _init_servos(self):
        self.servos = {}
        for name, data in self.config.servos.items():
            if data["pca9685"] == 1:
                srv_1 = servo.Servo(self.pca_1.channels[data["channel"]])
                srv_1.set_pulse_width_range(min_pulse= data["min_pulse"], max_pulse= data["max_pulse"])
                self.servos[name] = srv_1
            elif data["pca9685"] == 2:
                srv_2 = servo.Servo(self.pca_2.channels[data["channel"]])
                srv_2.set_pulse_width_range(min_pulse= data["min_pulse"], max_pulse= data["max_pulse"])
                self.servos[name] = srv_2
            else:
                print("Solo es la configuracion de dos placas")
                
    def set_servo_anglea(self, name, logical_angle_deg):
        """Envia un angulo logico (grados) al servo fisico.

        Args:
            name (string): Nombre de la articulacion. Ej. FL_hip_roll
            logical_angle_deg (float): Angulo en grados.
        """
        self.servos[name].angle = logical_angle_deg
    
    def set_leg_angles(self, leg_id, angles_deg):
        """Enviar una lista(list) angulos a una pata especifica.

        Args:
            leg_id (string): Nombre de pata del robot: FL, FR, RL, RR.
            angles_deg (list): Angulos de cada pata [hip_roll, hip_pitch, knee]
        """
        for name, angle in zip(self.config.leg_map[leg_id], angles_deg):
            self.set_servo_angle(name,angle)            
        
    def deinit(self):
        self.pca_1.deinit()
        self.pca_2.deinit()
        
if __name__ == "__main__":
    servo = ServoController()