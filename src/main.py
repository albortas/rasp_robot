import time

from src.control.control_interface import ControlInterface
from src.control.gait_inteface import GaitInteface
from src.controller.PS4Controller import PS4Controller

# Estados
NEUTRAL = 0
STATIC_POSTURE = 1
GAIT_MODE = 2


def main():
    print("Iniciando control con toggle (L1 = postura)")

    # Inicializar módulos
    ps4 = PS4Controller()
    control = ControlInterface()
    gait = GaitInteface()

    # Limites
    MAX_ROLL = 0.4
    MAX_PITCH = 0.4
    MAX_YAW = 0.4

    # Estado Inicial
    current_mode = NEUTRAL
    control.go_to_neutral()
    print("Modo: NEUTRAL")

    # Para detectar flanco ascendete (presion unica)
    last_buttons = [False] * 14

    # Variables de control
    last_print_time = time.time()
    PRINT_INTERVAL = 0.5  # segundos

    try:
        while True:
            try:
                state = ps4.get_joystick_state()
            except Exception as e:
                if current_mode != NEUTRAL:
                    print(f"Error con el mando PS4: {e}. Volviendo a NEUTRAL por seguridad.")
                    current_mode = NEUTRAL
                    control.go_to_neutral()
                time.sleep(1.0)
                continue

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
            
            # Toggle: R1 (5) -> marcha
            elif buttons[5] and not last_buttons[5]:
                if current_mode == GAIT_MODE:
                    current_mode = NEUTRAL
                    control.go_to_neutral()
                    print("Modo: NEUTRAL")
                else:
                    current_mode = GAIT_MODE
                    print("Modo: MARCHA (R1 para salir)")
                    
            # Comprobar si es momento de imprimir
            current_time = time.time()
            should_print = (current_time - last_print_time) >= PRINT_INTERVAL

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
                if should_print:
                    print(f"[Postura] Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")
                    last_print_time = current_time
            
            elif current_mode == GAIT_MODE:
                vel_x = -axes[1] * gait.max_vel_xy
                vel_y = -axes[0] * gait.max_vel_xy
                yaw_rate = axes[3] * gait.max_yaw_rate
                T_bf = gait.compute_gait(vel_x, vel_y, yaw_rate)
                angles = control.get_gait_angles(T_bf)
                control.send_joint_angles(angles)
                if should_print:
                    print(f"[Marcha] vx: {vel_x:.2f}, vy: {vel_y:.2f}, yaw: {yaw_rate:.2f}")
                    last_print_time = current_time
            
            last_buttons = buttons[:]
            time.sleep(0.02)
                
    except KeyboardInterrupt:
        print("Apagando robot...")
    finally:
        print("Volviendo a neutral...")
        control.shutdown()
        print("¡Sistema detenido!")

if __name__ == "__main__":
    main()