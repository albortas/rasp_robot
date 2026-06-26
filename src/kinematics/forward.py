from typing import Tuple, List, Union
import numpy as np


class Forward:
    def __init__(
        self,
        L1: float = 0.0615,
        L2: float = 0.108,
        L3: float = 0.1302,
    ):
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

    def solve(
        self, legtype: str, theta: Union[List[float], np.ndarray]
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if legtype == "LEFT":
            return self._forward_kinematic(theta, side=1)
        else:
            return self._forward_kinematic(theta, side=-1)

    def _forward_kinematic(
        self, theta: Union[List[float], np.ndarray], side: int
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Posicion de los sistemas de coordenadas cinemática directa"""

        # Sistema de coordenadas S_1
        p1 = np.array(
            [
                0,
                side * self.L1 * np.cos(theta[0]),
                side * self.L1 * np.sin(theta[0]),
            ]
        )

        # Sistema de coordenadas S_2
        p2 = np.array(
            [
                -self.L2 * np.sin(theta[1]),
                side * self.L1 * np.cos(theta[0])
                + self.L2 * np.sin(theta[0]) * np.cos(theta[1]),
                side * self.L1 * np.sin(theta[0])
                - self.L2 * np.cos(theta[0]) * np.cos(theta[1]),
            ]
        )

        # Sistema de coordenadas S_3
        p3 = np.array(
            [
                (-self.L2 * np.sin(theta[1]) - self.L3 * np.sin(theta[1] + theta[2])),
                (
                    side * self.L1 * np.cos(theta[0])
                    + self.L2 * np.sin(theta[0]) * np.cos(theta[1])
                    + self.L3 * np.sin(theta[0]) * np.cos(theta[1] + theta[2])
                ),
                (
                    side * self.L1 * np.sin(theta[0])
                    - self.L2 * np.cos(theta[0]) * np.cos(theta[1])
                    - self.L3 * np.cos(theta[0]) * np.cos(theta[1] + theta[2])
                ),
            ]
        )

        return p1, p2, p3

