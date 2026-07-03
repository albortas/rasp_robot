import numpy as np
import copy

from src.gait.bezier_gait import BazierGait
from src.kinematics.robot_model import RobotModel


class GaitInteface:
    def __init__(self, Tswing=0.2, dt=0.02) -> None:
        self.gait_gen = BazierGait(Tswing=Tswing, dt=dt)
        self.robot_model = RobotModel()
        self.T_bf_base = copy.deepcopy(self.robot_model.WorldToFoot)
        self.dt = dt

        # Parametros ajustables
        self.max_vel_xy = 0.15  # m/s
        self.max_yaw_rate = 1.0  # rad/s
        self.clearance_height = 0.04
        self.penetration_depth = 0.005

    def compute_gait(self, vel_x, vel_y, yaw_rate):
        L = np.hypot(vel_x, vel_y) / 2.0
        LateralFraction = np.arctan2(vel_x, vel_y) if L > 0.01 else 0.0
        vel = np.hypot(vel_x, vel_y)
        return self.gait_gen.GenerateTrayectory(
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
