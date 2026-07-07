import time
from enum import IntEnum

from src.control.control_interface import ControlInterface
from src.control.gait_inteface import GaitInteface
from src.controller.PS4Controller import PS4Controller
from src.hardware.servos_power_controller import ServosPowerController


class RobotMode(IntEnum):
    NEUTRAL = 0
    STATIC_POSTURE = 1
    GAIT_MODE = 2


class Robot:
    MAX_ROLL = 0.4
    MAX_PITCH = 0.4
    MAX_YAW = 0.4

    PRINT_INTERVAL = 0.5
    LOOP_PERIOD = 0.02

    def __init__(self):
        self.ps4 = PS4Controller()
        self.control = ControlInterface()
        self.gait = GaitInteface()
        self.power_controller = ServosPowerController()
        self.current_mode = RobotMode.NEUTRAL
        self._last_print_time = time.time()

        print("Iniciando control robot")
        self.control.go_to_neutral()
        print("Modo: NEUTRAL")
        self._run()

    def _run(self):
        last_buttons: tuple[bool, ...] = (False,) * 14

        try:
            while True:
                try:
                    state = self.ps4.get_joystick_state()
                except Exception as e:
                    if self.current_mode != RobotMode.NEUTRAL:
                        print(f"Error con el mando PS4: {e}. Volviendo a NEUTRAL por seguridad")
                        self._switch_mode(RobotMode.NEUTRAL)
                    time.sleep(1.0)
                    continue

                axes = state["axes"]
                buttons = tuple(state["buttons"])

                if self._rising_edge(buttons, last_buttons, 9):
                    self.power_controller.toggle()
                    self._switch_mode(RobotMode.NEUTRAL)

                if self.power_controller.state:
                    self._handle_mode_buttons(buttons, last_buttons)
                    self._execute_mode(axes)
                    last_buttons = buttons
                    time.sleep(self.LOOP_PERIOD)
                else:
                    print("!Habilitar el control de potencia de los servos!")
                    print("Presiona Options")
                    time.sleep(1)

        except KeyboardInterrupt:
            print("Apagando robot...")
        finally:
            print("Volviendo a neutral...")
            self.control.shutdown()
            print("Sistema detenido!")

    def _rising_edge(self, buttons: tuple[bool, ...], last: tuple[bool, ...], idx: int) -> bool:
        return buttons[idx] and not last[idx]

    def _switch_mode(self, mode: RobotMode):
        self.current_mode = mode
        if mode == RobotMode.NEUTRAL:
            self.control.go_to_neutral()

    def _handle_mode_buttons(self, buttons: tuple[bool, ...], last: tuple[bool, ...]):
        # L0 (4) -> postura estatica
        if self._rising_edge(buttons, last, 4):
            if self.current_mode == RobotMode.STATIC_POSTURE:
                self._switch_mode(RobotMode.NEUTRAL)
                print("Modo: NEUTRAL")
            else:
                self.current_mode = RobotMode.STATIC_POSTURE
                print("Modo: POSTURA ESTATICA (L0 para salir)")

        # R0 (5) -> marcha
        elif self._rising_edge(buttons, last, 5):
            if self.current_mode == RobotMode.GAIT_MODE:
                self._switch_mode(RobotMode.NEUTRAL)
                print("Modo: NEUTRAL")
            else:
                self.current_mode = RobotMode.GAIT_MODE
                print("Modo: MARCHA (R0 para salir)")

    def _execute_mode(self, axes: list[float]):
        current_time = time.time()
        should_print = (current_time - self._last_print_time) >= self.PRINT_INTERVAL

        if self.current_mode == RobotMode.NEUTRAL:
            self.control.go_to_neutral()

        elif self.current_mode == RobotMode.STATIC_POSTURE:
            roll = -axes[0] * self.MAX_ROLL
            pitch = axes[-1] * self.MAX_PITCH
            yaw = axes[3] * self.MAX_YAW
            angles = self.control.get_posture_angles(roll, pitch, yaw)
            self.control.send_joint_angles(angles)
            if should_print:
                print(f"[Postura] Roll: {roll:.1f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
                self._last_print_time = current_time

        elif self.current_mode == RobotMode.GAIT_MODE:
            vel_x = -axes[1] * self.gait.max_vel_xy
            vel_y = -axes[0] * self.gait.max_vel_xy
            yaw_rate = axes[3] * self.gait.max_yaw_rate
            T_bf = self.gait.compute_gait(vel_x, vel_y, yaw_rate)
            angles = self.control.get_gait_angles(T_bf)
            self.control.send_joint_angles(angles)
            if should_print:
                print(f"[Marcha] vx: {vel_x:.1f}, vy: {vel_y:.2f}, yaw: {yaw_rate:.2f}")
                self._last_print_time = current_time


if __name__ == "__main__":
    Robot()
