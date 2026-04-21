import numpy as np
import copy
from src.kinematics.RobotModel import RobotModel
from src.hardware.ServoController import ServoController

class ControlInterface:
    def __init__(self):
        self.robot_model = RobotModel()
        self.servo_controller = ServoController()
        self.T_bf_base = copy.deepcopy(self.robot_model.WorldToFoot)
        self.leg_order = ["FL", "FR", "RL", "RR"]

    def send_joint_angles(self, joint_angles_rad):
        angles_deg = np.degrees(joint_angles_rad)
        for i, leg in enumerate(self.leg_order):
            self.servo_controller.set_leg_angles(leg, angles_deg[i])
        
    def get_neutral_angles(self):
        return self.robot_model.IK(
            rpy=np.array([0, 0, 0]),
            pos=np.array([0, 0, 0]),
            T_bf=self.T_bf_base
        )
    
    def get_posture_angles(self, roll, pich, yaw):
        rpy = np.array([roll, pich, yaw])
        return self.robot_model.IK(
            rpy=rpy,
            pos=np.array([0, 0, 0]),
            T_bf=self.T_bf_base
        )

    def get_gait_angles(self, T_bf_gait):
        return self.robot_model.IK(
            np.array([0, 0, 0]),
            np.array([0, 0, 0]),
            T_bf_gait
        )
    
    def go_to_neutral(self):
        angles = self.get_neutral_angles()
        self.send_joint_angles(angles)
        
    def shutdown(self):
        self.go_to_neutral()
        self.servo_controller.deinit()