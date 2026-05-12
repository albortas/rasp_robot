from pick import pick
from src.utils.servo_repository import ServoRepository
from src.utils.toml_loader import TomlLoader
from src.utils.servo_configurator import ServoConfigurator

try:
    from src.hardware.hardware_manager import HardwareManager
    HARDWARE_AVAILABLE = True
    SIMULATION_ERROR = ""
except Exception as e:
    HARDWARE_AVAILABLE = False
    SIMULATION_ERROR = str(e)

class CalibratorApp:
    def __init__(self):
        self.repo = ServoRepository()
        self.loader = TomlLoader(self.repo)
        self.loader.load_from_file()
        
        self.configurator = ServoConfigurator(self.repo, self.loader)
        
        self.hw = None
        if HARDWARE_AVAILABLE:
            try:
                self.hw = HardwareManager(self.loader, self.repo)
            except Exception as e:
                print(f"Error inicializando Hardware: {e}")
                
        # Forzar uso de PCA 1 para calibracion
        self.configurator.update_channels(1)
        
        # Agrupar por patas
        self._leg_map()
        
    def _leg_map(self):
        self.legs = ["FL", "FR", "RL", "RR"]
        self.joints = {leg: [] for leg in self.legs}
        for name in self.repo.get_servo_names():
            leg = name[:2]
            if leg in self.joints:
                self.joints[leg].append(name)

    def main_menu(self):
        title = "=== CALIBRACION INTERACTIVA DE SERVOS ==="
        if not self.hw:
            title += f"\n[MODO SIMULACION - Hardware I2C no detectado: {SIMULATION_ERROR}]"
        else:
            title += "\n[MODO REAL - Hardware I2C activo]"
        title += "\n(Todos los canales han sido forzados temporalmente a PCA 1 para calibracion)"
        
        options = self.legs + ["Salir"]
        
        while True:
            option, index = pick(options, title)
            if index < len(self.legs):
                self.leg_menu(option)
            else:
                if self.hw:
                    self.hw.deinit()
                print("Calibracion finalizada.")
                break

    def leg_menu(self, leg):
        title = f"Selecciona la articulacion de la pata {leg}:"
        joints = self.joints.get(leg, [])
        options = joints + ["Volver"]
        
        while True:
            option, index = pick(options, title)
            if index < len(joints):
                self.action_menu(option)
            else:
                break

    def action_menu(self, joint_name):
        cursor = 0
        while True:
            servo = self.repo.select_servo(joint_name)
            title = (f"=== Servo: {joint_name} ===\n"
                     f"Offset actual: {servo.offset}\n"
                     f"Inversion actual: {servo.invert_direction}\n"
                     f"Rest Angle: {servo.rest_angle}\n"
                     "-------------------------")
            
            options = [
                "1. Ir a Posicion de Reposo (rest_angle)",
                "2. Ajuste Fino de Offset",
                "3. Invertir Direccion",
                "4. Test de Movimiento (Presets)",
                "0. Volver"
            ]
            _, index = pick(options, title, default_index=cursor)
            cursor = index
            
            if index == 0:
                self._send_angle(joint_name, servo.rest_angle)
            elif index == 1:
                self.fine_tuning_menu(joint_name)
            elif index == 2:
                self._toggle_inversion(joint_name)
            elif index == 3:
                self.preset_menu(joint_name)
            elif index == 4:
                break

    def fine_tuning_menu(self, joint_name):
        cursor = 0
        while True:
            servo = self.repo.select_servo(joint_name)
            title = (f"--- Ajuste Fino: {joint_name} ---\n"
                     f"Offset actual: {servo.offset}\n"
                     f"Usando rest_angle: {servo.rest_angle} como base de calibracion.")
            options = ["Incrementar Offset (+1)", "Decrementar Offset (-1)", "Volver"]
            
            _, index = pick(options, title, default_index=cursor)
            cursor = index
            
            if index == 0:
                new_offset = servo.offset + 1
                self.configurator.update_offset(joint_name, new_offset)
                self.loader.synchronize() # Guardar cambios en disco
                self._send_angle(joint_name, servo.rest_angle)
            elif index == 1:
                new_offset = servo.offset - 1
                self.configurator.update_offset(joint_name, new_offset)
                self.loader.synchronize()
                self._send_angle(joint_name, servo.rest_angle)
            elif index == 2:
                break

    def _toggle_inversion(self, joint_name):
        servo = self.repo.select_servo(joint_name)
        new_val = not servo.invert_direction
        self.configurator.update_invert_direction(joint_name, new_val)
        self.loader.synchronize()
        self._send_angle(joint_name, servo.rest_angle)

    def preset_menu(self, joint_name):
        cursor = 2 # Default 90
        while True:
            title = f"--- Test Presets: {joint_name} ---"
            options = ["Ir a 0", "Ir a 45", "Ir a 90", "Ir a 135", "Ir a 180", "Volver"]
            
            _, index = pick(options, title, default_index=cursor)
            cursor = index
            
            if index == 0: self._send_angle(joint_name, 0)
            elif index == 1: self._send_angle(joint_name, 45)
            elif index == 2: self._send_angle(joint_name, 90)
            elif index == 3: self._send_angle(joint_name, 135)
            elif index == 4: self._send_angle(joint_name, 180)
            elif index == 5: break

    def _send_angle(self, joint_name, angle):
        if self.hw:
            self.hw.set_angle(joint_name, angle)

if __name__ == "__main__":
    app = CalibratorApp()
    app.main_menu()
