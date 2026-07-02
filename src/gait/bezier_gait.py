from math import comb
import numpy as np


class BazierGait:
    # Estos binarios
    STANCE = 0  # Contacto con el suelo
    SWING = 1  # Pierna en el aire

    def __init__(self, dSref=[0.0, 0.0, 0.5, 0.5], Tswing=0.2) -> None:
        # Desfase de fase por pata: [FL, FR, RL, RR]
        self.dSref = dSref
        # La pata de referencia es FL, siempre 0
        self.ref_idx = 0.0  # Pata de referencia
        # Tiempo transcurrido desde el último touchdown
        self.time_since_last_TD = 0.0
        self.StanceSwing = self.STANCE  # Modo de la trayectoria
        # Valor de fase en la fase de swing [0, 1] de la pata de referencia
        self.SwRef = 0.0
        # Indica si la pata de referencia ha tocado el suelo
        self.TD = False
        self.TD_time = 0.0  # Tiempo de touchdown
        self.time = 0.0  # Tiempo total transcurrido
        self.Tswing = Tswing  # Tiempo de SWING
        self.NumControlPoints = 11

    def Get_ti(self, index, Tstride):
        """
        Obtiene en indice temporal para una pat indivual
        """
        if index == self.ref_idx:
            self.dSref[index] = 0.0
        return self.time_since_last_TD - self.dSref[index] * Tstride

    def GetPhase(self, index, Tstance, Tswing):
        """
        Obtener la fase de una pata indivual.
        """
        StanceSwing = self.STANCE
        Sw_phase = 0.0

        Tstride = Tstance + Tswing
        ti = self.Get_ti(index, Tstride)

        if ti < -Tswing:
            ti += Tstride

        # STANCE
        if ti >= 0.0 and ti <= Tstance:
            StanceSwing = self.STANCE
            if Tstance == 0.0:
                Stnphase = 0.0
            else:
                Stnphase = ti / float(Tstance)

            if index == self.ref_idx:
                self.StanceSwing = StanceSwing
            return Stnphase, StanceSwing

        # SWING
        elif ti >= -Tswing and ti < 0.0:
            StanceSwing = self.SWING
            Sw_phase = (ti + Tswing) / Tswing
        elif ti > Tstance and ti <= Tstride:
            StanceSwing = self.SWING
            Sw_phase = (ti - Tstance) / Tswing

        # Contacto con el suelo al final de SWING
        if Sw_phase >= 1.0:
            Sw_phase = 1.0

        if index == self.ref_idx:
            self.StanceSwing = StanceSwing
            self.SwRef = Sw_phase

            # Contacto con el suelo de la pierna de referencia al final
            # del SWING
            if self.SwRef >= 0.999:
                self.TD = True
        return Sw_phase, StanceSwing

    def CheckTouchDown(self):
        """
        Verifica si ha ocurrido un touchdown de la pata de referencia,
        y si esto justifica reiniciar el tiempo de touchdown.
        """
        if self.SwRef >= 0.9 and self.TD:
            self.TD_time = self.time
            self.TD = False
            self.SwRef = 0.0

    def Increment(self, dt, Tstride):
        """
        Incrementa el reloj interno del generador de marcha de Bézier.
        """
        self.CheckTouchDown()
        self.time_since_last_TD = self.time - self.TD_time
        if self.time_since_last_TD > Tstride:
            self.time_since_last_TD = Tstride
        elif self.time_since_last_TD < 0.0:
            self.time_since_last_TD = 0.0

        # Incrementaremos el tiempo al final
        # en caso de que acabe de ocurrir un touchdown
        self.time += dt

        # Si Tstride = Tswing, entonces Tstance = 0
        # REINICIAMOS TODO
        if Tstride < self.Tswing + dt:
            self.time = 0.0
            self.time_since_last_TD = 0.0
            self.TD_time = 0.0
            self.SwRef = 0.0

    def BinomialCoefficient(self, k):
        """
        Resuelve el teorema del binomio dado el numero de un punto de Bezier
        en relacion con el numero total de puntos.
        """
        return comb(self.NumControlPoints, k)

    def BernSteinPoly(self, t, k, point):
        """
        Calcula el punto en el polinimio de Bernestein
        basado el la fase (0 -> 1), el numero del punto (0 - 11),
        y el valor del punto de control
        """
        return (
            point
            * self.BinomialCoefficient(k)
            * np.power(t, k)
            * np.power(1 - t, self.NumControlPoints - k)
        )

    def BezierSwing(self, phase, L, LateralFraction, cleareance_height=0.04):
        """
        Calcula las coordenadas del paso para el periode de Bezier (swing)

        :param phase: Fase actual de la trayectoria
        :param L: Longitud del paso
        :param LateralFraction: Determina cuan lateral es el movimiento
        :param cleareance_height: Altura de despeje del pie durante la fase swing

        :return: Coordenada x, y, z del pie relativas al cuerpo sin modificar
        """
        # Coordenadas polares de la pata
        X_POLAR = np.cos(LateralFraction)
        Y_POLAR = np.sin(LateralFraction)

        # Puntos de la curva de Bezier (12 pts)
        # NOTA: L es la MITAD de la longitud del paso
        # Componente hacia adelante

        STEP = np.array(
            [
                -L,  # Punto de control 0, mitad de la longitud
                -L
                * 1.4,  # Punto de control 1, diferencia entre 1 y 0 = velocidad de elvacion en X
                # Punto de control 2, 3, 4 superpuestos par el cambio de direccion despues de seguimiento
                -L * 1.5,
                -L * 1.5,
                -L * 1.5,
                0.0,  # Cambio de aceleracion durante la protraccion
                0.0,  # Por eso incluimos tre puntos de control superpuestos 5, 6 y 7
                L * 1.5,  # Cambio de direccion para la retraccion de la pata
                L * 1.5,  # en swing, requiere dos puntos superpuestos 8, 9.
                L * 1.4,  # Velocidad de retraccion de la pata en swing = Ctrl 11 - 10
                L,
            ]
        )

        # Consideramos movimientos laterales multiplicando por la coordenadas.
        # LateralFraction cambia el movimiento de la pata desde X hacia Y+ o Y-
        X = STEP * X_POLAR
        Y = STEP * Y_POLAR

        # Componete vertical
        Z = np.array(
            [
                0.0,  # Puntos de control doblemente superpuestos para velocidad
                0.0,  # cero de elevacion Velocidad repecto a la cadera (Pts 0 y 1)
            ]
        )
