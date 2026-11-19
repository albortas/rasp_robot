import numpy as np
import copy

from src.gait.bezier_gait import BazierGait
from src.kinematics.robot_model import RobotModel


class GaitInteface:
    def __init__(self, Tswing=0.25, dt=0.02) -> None:
        self.gait_gen = BazierGait(Tswing=Tswing, dt=dt)
        self.robot_model = RobotModel()
        self.T_bf_base = copy.deepcopy(self.robot_model.WorldToFoot)
        self.dt = dt

        # Parametros ajustables
        self.max_vel_xy = 0.07  # m/s
        self.max_yaw_rate = 0.5  # rad/s
        self.clearance_height = 0.04
        self.penetration_depth = 0.005

    def compute_gait(self, vel_x, vel_y, yaw_rate):
        """
        Convierte la orden de velocidad del mando en parametros del generador.

        :param vel_x: Velocidad lineal en X (m/s).
        :param vel_y: Velocidad lineal en Y (m/s).
        :param yaw_rate: Velocidad angular en Z (rad/s).
        :return: Diccionario T_bf con la posicion deseada de cada pie.
        """
        vel_x = float(vel_x)
        vel_y = float(vel_y)
        yaw_rate = float(np.clip(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate))

        vel = float(np.hypot(vel_x, vel_y))
        if vel > 1e-4:
            # La direccion se decide con el vector completo, no con la
            # semizancada: si no, los comandos laterales lentos se
            # convertian en avance.
            vel = min(vel, self.max_vel_xy)
            L = vel / 2.0
            LateralFraction = float(np.arctan2(vel_y, vel_x))
        else:
            vel = 0.0
            L = 0.0
            LateralFraction = 0.0

        return self.gait_gen.GenerateTrajectory(
            L=L,
            LateralFraction=LateralFraction,
            YawRate=yaw_rate,
            vel=vel,
            T_bf_=self.T_bf_base,
            T_bf_curr=self.T_bf_base,
            clearance_height=self.clearance_height,
            penetration_depth=self.penetration_depth,
            contacts=[1, 1, 1, 1],
            dt=self.dt,
        )
