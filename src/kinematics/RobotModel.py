import numpy as np

from src.kinematics.Inverse import Inverse
from src.kinematics.LieAlgebra import (
    RpToTrans,
    TransToRp,
    TransInv,
    RPY
)

class RobotModel:
    def __init__(
        self,
        L1 = 0.0580,
        L2 = 0.1080,
        L3 = 0.1385,
        hip_x = 0.192,
        hip_y = 0.078,
        foot_x = 0.192,
        foot_y = 0.194,
        height = 0.135
    ):
        # Parametros de la pierna
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        
        # Iniciar un objeto Inverse
        self.inverse = Inverse()
        # Distancia entre caderas
        # Longitud
        self.hip_x = hip_x
        # Ancho
        self.hip_y = hip_y
        
        # Distancia punto medio entre las caderas
        self.m_hip_x = hip_x/2.0
        self.m_hip_y = hip_y/2.0
        
        # Distancia entre los pies
        # Longitud
        self.foot_x = foot_x
        # Ancho
        self.foot_y = foot_y
        
        # Distancia punto medio entre los pies
        self.m_foot_x = foot_x/2.0
        self.m_foot_y = foot_y/2.0        
        
        # Altura del cuerpo
        self.height = height
        
        # Puntos desde el centroide del robot a caderas/hombros(hip/shoulder)
        ph_FL = np.array([self.m_hip_x, self.m_hip_y, 0])
        ph_FR = np.array([self.m_hip_x, -self.m_hip_y, 0])
        ph_RL = np.array([-self.m_hip_x, self.m_hip_y, 0])
        ph_RR = np.array([-self.m_hip_x, -self.m_hip_y, 0])

        # Puntos desde el centroide del robot a pies(foot)
        pf_FL = np.array([self.m_foot_x, self.m_foot_y, -self.height])
        pf_FR = np.array([self.m_foot_x, -self.m_foot_y, -self.height])
        pf_RL = np.array([-self.m_foot_x, self.m_foot_y, -self.height])
        pf_RR = np.array([-self.m_foot_x, -self.m_foot_y, -self.height])

        # Diccionario para almacenar soluciones de Cinematica Inversa
        self.Legs = {"FL": "LEFT", "FR": "RIGHT", "RL": "LEFT", "RR": "RIGHT"}

        # Transfomaciones de la cadera en relación al centroide del cuerpo
        Rwb = np.eye(3) # Matriz identidad
        self.WorldToHip = {}    # Diccionario Vacio
        self.WorldToHip["FL"] = RpToTrans(Rwb, ph_FL)
        self.WorldToHip["FR"] = RpToTrans(Rwb, ph_FR)
        self.WorldToHip["RL"] = RpToTrans(Rwb, ph_RL)
        self.WorldToHip["RR"] = RpToTrans(Rwb, ph_RR)
        
        # Transfomaciones de los pies en relación al centroide del cuerpo 
        self.WorldToFoot = {}   # Diccionario Vacio
        self.WorldToFoot["FL"] = RpToTrans(Rwb, pf_FL)
        self.WorldToFoot["FR"] = RpToTrans(Rwb, pf_FR)
        self.WorldToFoot["RL"] = RpToTrans(Rwb, pf_RL)
        self.WorldToFoot["RR"] = RpToTrans(Rwb, pf_RR)
        
    def HipToFoot(self, rpy, pos, T_bf):
        """
        Convierte la posición y orientación deseadas respecto a la posición
        inicial de Robot, con la transformación deseada de cuerpo a pie,
        en una transformación de cuerpo a cadera, que se utiliza para extraer
        y devolver el vector de cadera a pie

        :param orn: Un 3x1 np.array([]) con ángulos Roll, Pitch, Yaw del Robot
        :param pos: Un 3x1 np.array([]) con las coordenada X, Y, Z del Robot
        :param T_bf: Diccionario de las transformaciones deseadas de cuerpo a pie
        :return: Vector de cadera a pie para cada pierna
        """
        
        HipToFoot_Dic = {}

        # wb -> world to body
        R_wb, _ = TransToRp(RPY(rpy[0], rpy[1], rpy[2]))
        p_wb = pos
        T_wb = RpToTrans(R_wb, p_wb)
        
        # wh -> world to hip
        for key, T_wh in self.WorldToHip.items():
            # ORDEN: FL, FR, RL, RR
            
            # Paso 1: obtener T_bh para cada pierna
            
            # bw -> body to world
            T_bw = TransInv(T_wb)

            # bh -> body to hip
            T_bh = T_bw @ T_wh
            
            # METODO ADICICION DE VECTORES
            # # bf -> body to foot
            _, p_bf = TransToRp(T_bf[key])
            
            _, p_bh = TransToRp(T_bh)
            p_hf0 = p_bf - p_bh

            # METODO DE MULTIPLICACION DE TRANSFORMADA
            # hb -> hip to body
            T_hb = TransInv(T_bh)
            T_hf = T_hb @ T_bf[key]
            _, p_hf1 = TransToRp(T_hf)
            
            # Los dos metodos son iguales
            if p_hf0.all() != p_hf1.all():
                print(f"{p_hf0} no es igual {p_hf1}")

            HipToFoot_Dic[key] = p_hf1
        
        return HipToFoot_Dic
    
    def IK(self, rpy, pos, T_bf):
        """
        Utiliza HipToFoot() para convertir la posión y
        orientación deseadas respecto a la posión inicial del Robot
        en un vector de Cadera a Pie, que se introduce en el solucionador
        de la Cinematica Inversa.

        Finalmente, el solucionador de Cinematica Inversa devuelve los
        ángulos de la articulación resultante para cada pierna.

        :param orn: Un 3x1 np.array([]) con los ángulos de Roll, Pitch, Yaw del Robot.
        :param pos: Un 3x1 np.array([]) con las coordenadas X, Y, Z del Robot.
        :param T_bf: Diccionario de las transformaciones deseadas del Cuerpo a Pie.
        :return: Ángulos de articulación para cada articulación del Robot.
        """

        # 4 piernas, 3 articulaciones por pierna
        joint_angles = np.zeros((4, 3))

        HipToFoot = self.HipToFoot(rpy, pos, T_bf)

        for i, (key, p_hf) in enumerate(HipToFoot.items()):
            # ORDER: FL, FR, RL, RR

            # print("LEG: {} \t HipToFoot: {}".format(key, p_hf))

            # Step 3, compute joint angles from T_hf for each leg
            joint_angles[i, :] = self.inverse.solve(key, p_hf)

        # print("-----------------------------")

        return joint_angles
    
    

if __name__ == '__main__':
    print("Probando HipToFoot")
    rpy = [np.pi/4, 0, 0]
    pos = [0, 0 , 0]
    robot = RobotModel()
    T_bf = robot.WorldToFoot
    p_hfs = robot.HipToFoot(rpy, pos, T_bf)
    for clave, valor in p_hfs.items():
        print(clave)
        print(valor)
    angles = robot.IK(rpy, pos, T_bf)
    print(angles)
    