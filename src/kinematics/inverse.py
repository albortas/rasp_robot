from typing import Tuple, Optional
import numpy as np

from src.utils.logger import log


class Inverse:
    def __init__(
        self,
        L1: float = 0.0615,
        L2: float = 0.108,
        L3: float = 0.1302,
    ):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

    def get_domain(self, x: float, y: float, z: float) -> float:
        """Calcula el dominio de la pierna y lo limita en caso de incumplimiento

        :param x, y, z: Distancia de cadera a pie en cada dimension
        :return: Dominio de la pierna D.
        """
        D = (x**2 + y**2 + z**2 - self.L1**2 - self.L2**2 - self.L3**2) / (
            2 * self.L2 * self.L3
        )
        if D > 1 or D < -1:
            # Dominio Invalido
            log.debug("----- DOMINIO INVALIDO -----")
            D = np.clip(D, -1.0, 1.0)
            return float(D)
        else:
            return float(D)

    def solve(self, legtype, xyz_coord) -> np.ndarray:
        """Solucionador generico de cinematica inversa de piernas

        Args:
            xyz_coord (m): Distancia de cadera a pie de cada dimension
            return: Angulos articulares necesarios para la posición deseada
        """
        x = xyz_coord[0]
        y = xyz_coord[1]
        z = xyz_coord[2]
        D = self.get_domain(x, y, z)
        if legtype == "LEFT":
            return self._inverse_kinematic(x, y, z, D, side=1)
        else:
            return self._inverse_kinematic(x, y, z, D, side=-1)

    def _inverse_kinematic(
        self, x: float, y: float, z: float, D: float, side: int
    ) -> np.ndarray:
        """Solucionador de cinematica inversa de pierna izquierda

        Args:
            x, y, z (m): Distancia de cadera a pie en cada dimension
            D : Dominio de la pierna
        Return: Angulos articulares necesarios para la posicion deseada
        """
        AG_cuadrado = y**2 + z**2 - self.L1**2
        if AG_cuadrado < 0.0:
            log.warning("SQRT NEGATIVO")
            AG_cuadrado = 0.0
        AG = np.sqrt(AG_cuadrado)
        num = -(z / y + AG / (side * self.L1))
        den = 1 - (z / y) * (AG / (side * self.L1))
        theta1 = np.arctan(num / den)
        theta3 = np.arccos(D)
        theta2 = np.arctan2(x, AG) - np.arctan2(
            self.L3 * np.sin(theta3), self.L2 + self.L3 * np.cos(theta3)
        )
        return np.array([-theta1, -theta2, -theta3])

    def get_range(self, z: float) -> Optional[Tuple[float, float]]:
        lower_limit_base = self.L1**2 + (self.L2 - self.L3) ** 2
        upper_limit_base = self.L1**2 + (self.L2 + self.L3) ** 2

        # Efecto de las variables fijas en el plano YZ
        y = 0.0615  # metros
        x = 0.0
        sum_squares_yz = y**2 + z**2
        sum_squares_xz = x**2 + z**2

        # 1. Validacion de seguridad ¿El punto YZ está fuera del alcance máximo del robot?
        if sum_squares_yz > upper_limit_base:
            distance_yz = np.sqrt(sum_squares_yz)
            max_distance = np.sqrt(upper_limit_base)
            log.error(
                f"La combinación (y={y}, z={z}) está fuera del espacio de trabajo."
            )
            log.info(
                f"Distancia actual en YZ: {distance_yz:.4f} m | Máxima permitida total: {max_distance:.4f} m"
            )
            return None
        elif sum_squares_xz > upper_limit_base:
            distance_xz = np.sqrt(sum_squares_xz)
            max_distance = np.sqrt(upper_limit_base)
            log.error(
                f"La combinación (x={x}, z={z}) está fuera del espacio de trabajo."
            )
            log.info(
                f"Distancia actual en YZ: {distance_xz:.4f} m | Máxima permitida total: {max_distance:.4f} m"
            )
            return None

        # 2. Aplicación de las fórmulas con las constantes de control I y S
        L_x_axis = lower_limit_base - sum_squares_yz
        U_x_axis = upper_limit_base - sum_squares_yz

        L_y_axis = lower_limit_base - sum_squares_xz
        U_y_axis = upper_limit_base - sum_squares_xz

        # Filtro max(0, I) para evitar raíces cuadradas de números negativos
        L_filter_x_axis = max(0.0, L_x_axis)
        L_filter_y_axis = max(0.0, L_y_axis)

        # 3. Cálculo de los límites físicos reales para X
        x_min = np.sqrt(L_filter_x_axis)
        x_max = np.sqrt(U_x_axis)

        y_min = np.sqrt(L_filter_y_axis)
        y_max = np.sqrt(U_y_axis)

        # Mostrar resultados formateados en pantalla
        log.info(f"=== RANGOS VÁLIDOS PARA X (con y = {y} m, z = {z} m) ===")
        if x_min == 0:
            log.info("Rango continuo: El eje X puede cruzar por el centro (0).")
            log.info(
                f"Intervalo total de movimiento: [{-x_max:.4f} a {x_max:.4f}] metros"
            )
        else:
            log.warning("Zona muerta detectada: El eje X NO puede acercarse al centro.")
            log.info(f"Rango Positivo: [{x_min:.4f} a {x_max:.4f}] metros")
            log.info(f"Rango Negativo: [{-x_max:.4f} a {-x_min:.4f}] metros")

        log.info(f"=== RANGOS VÁLIDOS PARA Y (con x = {x} m, z = {z} m) ===")
        if y_min == 0:
            log.info("Rango continuo: El eje Y puede cruzar por el centro (0).")
            log.info(
                f"Intervalo total de movimiento: [{-y_max:.4f} a {y_max:.4f}] metros"
            )
        else:
            log.warning("Zona muerta detectada: El eje Y NO puede acercarse al centro.")
            log.info(f"Rango Positivo: [{y_min:.4f} a {y_max:.4f}] metros")
            log.info(f"Rango Negativo: [{-y_max:.4f} a {-y_min:.4f}] metros")
        return (float(x_min), float(x_max))


if __name__ == "__main__":
    np.set_printoptions(precision=3, suppress=True)
    ik = Inverse()
    position = [0, 0.0615, -0.135]
    theta = np.degrees(ik.solve("RIGHT", position))
    log.info(theta)
    ik.get_range(-0.15)

