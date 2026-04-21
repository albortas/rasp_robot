from pick import pick

class ServoSimple:
    def __init__(self, config):
        self.config = config
        self._leg_map()
        self._joints_rest_angle()

    def _leg_map(self):
        legs = []
        joints = []
        for key, value in self.config.leg_map.items():
            legs.append(key)
            joints.append(value)
        self.legs = legs
        self.joints = joints
    
    def _joints_rest_angle(self):
        rest_angles = {}
        for key, value in self.config.servos.items():
            rest_angle = value["rest_angle"]
            rest_angles[key] = rest_angle
        self.state_robot = rest_angles

    def main(self):
        title = "CONFIGURADOR DE ROBOT CUADRUPEDO (12 DOF)"
        options = [
            "FL (Front Left)",
            "FR (Front Right)",
            "RL (Rear Left)",
            "RR (Rear Right)",
            "Salir",
        ]

        while True:
            option, index = pick(options, title)
            if index < 4:
                self.main_joint(option, index)
            else:
                print("¡Programa Terminado!")
                exit()

    def main_joint(self, leg, num_leg):
        title = f"Articulaciones de {leg}"
        joints = self.joints[num_leg]
        options = joints + ["Volver a Seleccion de Extremidades"]
        while True:
            option, index = pick(options, title)
            if index < 3:
                self.main_angle(leg, option)
            else:
                break

    def main_angle(self, leg, joint):
        current_angle = self.state_robot[joint]
        title = f"""--- {leg} > {joint} (Actual: {current_angle}º ---)
        ¿Como controlarlo?"""
        options = ["Ajuste Fino", "Ajuste Preestablecido", "Volver a Articulaciones"]
        _, index = pick(options, title)
        if index == 0:
            self.main_tuning_fine(leg, joint)
        elif index == 1:
            self.main_presets(leg, joint)

    def main_tuning_fine(self, leg, joint):
        current_angle = self.state_robot[joint]
        cursor = 0
        while True:
            title = f"--- Ajuste Fino: {leg} > {joint} ({current_angle}) ---"
            options = ["Incrementar 1 grado", "Decrementar 1 grado", "Volver"]
            _, index = pick(options, title, default_index=cursor)

            if index == 0:
                if current_angle <= 180:
                    cursor = 0
                    current_angle += 1
                    self.send_angle_servo(joint, current_angle)
            elif index == 1:
                if current_angle > 0:
                    cursor = 1
                    current_angle -= 1
                    self.send_angle_servo(joint, current_angle)
            elif index == 2:
                break

    def main_presets(self, leg, joint):
        cursor = 2

        while True:
            current_angle = self.state_robot[leg][joint]
            title = f"--- Ajustes Preestablecidos: {leg} > {joint} (Angulo: {current_angle}) ---"
            options = [
                "Ir a 0 grados",
                "Ir a 45 grados",
                "Ir a 90 grados",
                "Ir a 135 grados",
                "Ir a 180 grados",
                "Volver",
            ]
            _, index = pick(options, title, default_index=cursor)

            if index == 0:
                self.send_angle_servo(joint, 0)
            elif index == 1:
                self.send_angle_servo(joint, 45)
            elif index == 2:
                self.send_angle_servo(joint, 90)
            elif index == 3:
                self.send_angle_servo(joint, 135)
            elif index == 4:
                self.send_angle_servo(joint, 180)
            elif index == 5:
                break

    def send_angle_servo(self, joint, angle):
        self.state_robot[joint] = angle


if __name__ == "__main__":
    from src.utils.config import Config

    config = Config()
    servo = ServoSimple(config)
    servo.main()