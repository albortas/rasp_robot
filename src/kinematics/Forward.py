import numpy as np


class Forward:
    def __init__(self, legtype="LEFT", L1=0.0685, L2=0.10805, L3=0.1385):
        self.legtype = legtype
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3

    def solve(self, theta):
        if self.legtype == "LEFT":
            return self.ForwardKinematic(theta, side=1)
        else:
            return self.ForwardKinematic(theta, side=-1)

    def ForwardKinematic(self, theta, side):
        """Posicion del los sistemas de coordenadas cinemática directa"""

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

        # Sistema de coordenadas S_2
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
