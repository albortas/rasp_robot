import time

# import numpy as np

from src.control.control_interface import ControlInterface
from src.controller.PS4Controller import PS4Controller

# Estados
NEUTRAL = 0
STATIC_POSTURE = 1


def main():
    print("Iniciando control con toggle (L1 = postura)")

    # Inicializar módulos
    ps4 = PS4Controller()
    control = ControlInterface()

    # Limites
    MAX_ROLL = 0.5
    MAX_PITCH = 0.5
    MAX_YAW = 0.5

    # Estado Inicial
    current_mode = NEUTRAL
    control.go_to_neutral()
    print("Modo: NEUTRAL")

    # Para detectar flanco ascendete (presion unica)
    last_buttons = [False] * 14

    try:
        while True:
            state = ps4.get_joystick_state()
            axes = state["axes"]
            buttons = state["buttons"]

            # Toggle: L1 (4) -> postura estatica
            if buttons[4] and not last_buttons[4]:
                if current_mode == STATIC_POSTURE:
                    current_mode = NEUTRAL
                    control.go_to_neutral()
                    print("Modo: Neutral")
                else:
                    current_mode = STATIC_POSTURE
                    print("Modo: POSTURA ESTATICA (L1 para salir)")
                    
            # === Ejecutar segun el modo actual
            if current_mode == NEUTRAL:
                angles = control.get_neutral_angles()
                control.send_joint_angles(angles)
            elif current_mode == STATIC_POSTURE:
                roll = -axes[0] * MAX_ROLL
                pitch = axes[1] * MAX_PITCH
                yaw = axes[3] * MAX_YAW
                angles = control.get_posture_angles(roll, pitch, yaw)
                control.send_joint_angles(angles)
                if int(time.time() * 2) % 2 == 0:
                    print(f"[Postura] Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Volviendo a neutral...")
        control.shutdown()
        print("¡Sistema detenido!")
