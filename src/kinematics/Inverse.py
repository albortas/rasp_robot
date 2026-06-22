import numpy as np


class Inverse:
    def __init__(self, legtype="LEFT", L1=0.0615, L2=0.108, L3=0.1302):
        self.legtype = legtype
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

    def get_domain(self, x, y, z):
        """Calcula el dominio de la pierna y lo limita en caso de incumplimiento

        :param x, y, z: Distancia de cadera a pie en cada dimension
        :return: Dominio de la pierna D.
        """
        D = (x**2 + y**2 + z**2 - self.L1**2 - self.L2**2 - self.L3**2) / (
            2 * self.L2 * self.L3
        )
        if D > 1 or D < -1:
            # Dominio Invalido
            print("----- DOMINIO INVALIDO -----")
            D = np.clip(D, -1.0, 1.0)
            return D
        else:
            return D

    def solve(self, xyz_coord):
        """Solucionador generico de cinematica inversa de piernas

        Args:
            xyz_coord (m): Distancia de cadera a pie de cada dimension
            return: Angulos articulares necesarios para la posición deseada
        """
        x = xyz_coord[0]
        y = xyz_coord[1]
        z = xyz_coord[2]
        D = self.get_domain(x, y, z)
        if self.legtype == "LEFT":
            return self.InverseKinematic(x, y, z, D, side=1)
        else:
            return self.InverseKinematic(x, y, z, D, side=-1)

    def InverseKinematic(self, x, y, z, D, side):
        """Solucionador de cinematica inversa de pierna izquierda

        Args:
            x, y, z (m): Distancia de cadera a pie en cada dimension
            D : Dominio de la pierna
        Return: Angulos articulares necesarios para la posicion deseada
        """
        AG_cuadrado = y**2 + z**2 - self.L1**2
        if AG_cuadrado < 0.0:
            print("SQRT NEGATIVO")
            AG_cuadrado = 0.0
        AG = np.sqrt(AG_cuadrado)
        num = -(z / y + AG / self.L1)
        den = 1 - (z / y) * (AG / self.L1)
        theta1 = np.arctan(num / den)
        theta3 = np.arccos(D)
        theta2 = np.arctan2(x, AG) - np.arctan2(
            self.L3 * np.sin(theta3), self.L2 + self.L3 * np.cos(theta3)
        )
        return np.array([-theta1, -theta2, -theta3])

    def get_range(self, y, z):
        lower_limit_base = self.L1**2 + (self.L2 - self.L3) ** 2
        upper_limit_base = self.L1**2 + (self.L2 + self.L3) ** 2

        # Efecto de las variables fijas en el plano YZ
        sum_squares_yz = y**2 + z**2

        # 1. Validacion de seguridad ¿El punto YZ está fuera del alcance máximo del robot?
        if sum_squares_yz > upper_limit_base:
            distance_yz = np.sqrt(sum_squares_yz)
            max_distance = np.sqrt(upper_limit_base)
            print(
                f"❌ Error: La combinación (y={y}, z={z}) está fuera del espacio de trabajo."
            )
            print(
                f"   Distancia actual en YZ: {distance_yz:.4f} m | Máxima permitida total: {max_distance:.4f} m"
            )
            return None

        # 2. Aplicación de las fórmulas con las constantes de control I y S
        I = lower_limit_base - sum_squares_yz
        S = upper_limit_base - sum_squares_yz

        # Filtro max(0, I) para evitar raíces cuadradas de números negativos
        I_filtrado = max(0.0, I)

        # 3. Cálculo de los límites físicos reales para X
        x_min = np.sqrt(I_filtrado)
        x_max = np.sqrt(S)

        # Mostrar resultados formateados en pantalla
        print(f"=== RANGOS VÁLIDOS PARA X (con y = {y} m, z = {z} m) ===")
        if x_min == 0:
            print("✅ Rango continuo: El eje X puede cruzar por el centro (0).")
            print(
                f"   Intervalo total de movimiento: [{-x_max:.4f} a {x_max:.4f}] metros"
            )
        else:
            print("⚠️ Zona muerta detectada: El eje X NO puede acercarse al centro.")
            print(f"   Rango Positivo: [{x_min:.4f} a {x_max:.4f}] metros")
            print(f"   Rango Negativo: [{-x_max:.4f} a {-x_min:.4f}] metros")

        return (x_min, x_max)


if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)
    ik = Inverse()
    position = [0, -0.0685, -0.23]
    theta = np.degrees(ik.solve(position))
    print(theta)
    ik.get_range(0.0615, 0.12)
