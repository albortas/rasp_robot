from math import comb
import copy
import numpy as np

from src.kinematics.lib_algebra import TransToRp

class BazierGait:
    # Estos binarios
    STANCE = 0  # Contacto con el suelo
    SWING = 1  # Pierna en el aire

    def __init__(self, dSref=[0.0, 0.0, 0.5, 0.5], dt = 0.01, Tswing=0.2) -> None:
        # Desfase de fase por pata: [FL, FR, RL, RR]
        self.dSref = dSref
        self.Tswing = Tswing  # Tiempo de SWING
        self.dt = dt    # Paso de tiempo
        self.Pre_fxyz = [0.0, 0.0, 0.0, 0.0]
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
        # Numero de puntos de control es n + 1 = 11 + 1 = 12
        self.NumControlPoints = 11
        # Almacena las fases de todas las patas
        self.Phase = self.dSref

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
                cleareance_height * 0.9,    # Tres puntos superpuestos par cambio de
                cleareance_height * 0.9,    # direccion de fuerza durante la transicion de
                cleareance_height * 0.9,    # seguimiento a protraccion (2, 3, 4)
                cleareance_height * 0.9,    # Puntos doblemente superpuestos par cambio de
                cleareance_height * 0.9,    # direccion de trayectoria durante protraccion (5 y 6)
                cleareance_height * 1.1,    # Altura maxima de despeje en el cento de la trayectoria Pt. 7
                cleareance_height * 1.1,    # Transicion suave de protraccion
                cleareance_height * 1.1,    # a retraccion, dos puntos de control (8 y 9)
                0.0,    # Puntos doblemente superpuestos para velocidad cero
                0.0,    # en el contacto con el suelo repecto repecto a la cadera (10 y 11)
            ]
        )

        stepX = 0.0
        stepY = 0.0
        stepZ = 0.0

        # Sumar los polinomio de Bernestein sobre los puntos de control
        for i in range(len(X)):
            stepX += self.BernSteinPoly(phase, i, X[i])
            stepY += self.BernSteinPoly(phase, i, Y[i])
            stepZ += self.BernSteinPoly(phase, i, Z[i])

        return stepX, stepY, stepZ

    def SineStance(self, phase, L, LateralFraction, penetration_depth = 0.0):
        """
        Calcula las coordenadas de paso para el periodo sinusoidad de STANCE

        :param phase: Fase actual de la trayectoria.
        :param L: Longitud de paso.
        :param LateralFraction: Determina cuan lateral es el movimiento.
        :param penetration_depth: Profundidad de penetracion del pie.

        :returns: Coordenada X, Y, Z del pie relativas al cuerpo.
        """
        X_POLAR = np.cos(LateralFraction)
        Y_POLAR = np.sin(LateralFraction)

        # Se mueve de +L a -L
        step = L * ( 1.0 - 2.0 * phase)
        stepX = step * X_POLAR
        stepY = step * Y_POLAR
        
        if L != 0.0:
            stepZ = - penetration_depth * np.cos(np.pi * (stepX + stepY)) / ( 2.0 * L)
        else:
            stepZ = 0.0

        return stepX, stepY, stepZ

    def YawCircle(self, T_bf, index):
        """
        Calcula la rotacion requerida del plano de la trayectoria
        para el movimiento de guiña (yaw)

        :param T_bf: Vector predeterminado de Cuerpo al Pie
        :param index: Indice del pie del contenedor

        :return: phi_arc, angulo de rotacion del plano necesario para
                 el movimento de yaw
        """

        # Magnitud del pie según el tipo de pata
        default_body_to_foot_magnitude = np.sqrt(T_bf[0] * 2 + T_bf[1] * 2)

        # Angulo de rotacion segun el tipo de pata
        default_body_to_foot_direction = np.arctan2(T_bf[1], T_bf[0])

        # Coordenadas anteriores de la pata relativas a las coordenadas
        # predeterminadas
        g_xyz = self.Pre_fxyz[index] - np.array([T_bf[0], T_bf[1], T_bf[2]])

        # Modulamos la magnitud para mantener el trazo del circulo
        g_map = np.sqrt(g_xyz[0] ** 2 + g_xyz ** 2)
        th_mod = np.arctan2(g_map, default_body_to_foot_magnitude)

        # Angulo corregido por el pie de la rotacion
        # FR y RL
        if index == 1 or index == 2:
            phi_arc = np.pi / 2.0 + default_body_to_foot_direction + th_mod
        # FL y RR
        else:
            phi_arc = np.pi / 2.0 - default_body_to_foot_direction + th_mod

        return phi_arc

    def SwingStep(self, phase, L, LateralFraction, YawRate, cleareance_height, T_bf, key, index):
        """
        Calcula las coordenadas del paso para el periodo de Bezeir SWING
        usando una conbinacion de coordenadas de paso lineal y rotacional
        inicialmente descompuesta de la entra del usuario:
        
        :param phase: Fase actual de la trayectoria.
        :param L: Longitud de paso.
        :param LateralFraction: Determina cuan lateral es el movimiento.
        :param YawRate: Tasa guiñada (yaw) deseada del cuerpo.
        :param cleareance_height: Altura de despeje del pie durante SWING.
        :param T_bf: Vector predeterminado de cuerpo a pie.
        :param key: Indica que pie se esta procesando.
        :param index: Indice del pie en el contenedor.

        :return: Coordenadas del pie relativas al cuerpo sin modificar.
        """
        # Angulo del pie para movimiento tangente al circulo en yaw
        phi_arc = self.YawCircle(T_bf, index)

        # Obtiene coordenadas del pie para el movimiento lineal
        X_delta_lin, Y_delta_lin, Z_delta_lin = self.BezierSwing(
                phase, L, LateralFraction, cleareance_height
                )

        X_delta_rot, Y_delta_rot, Z_delta_rot = self.BezierSwing(
                phase, YawRate, phi_arc, cleareance_height
                )

        coord = np.array([
            X_delta_lin + X_delta_rot,
            Y_delta_lin + Y_delta_rot,
            Z_delta_lin + Z_delta_rot
            ])

        self.Pre_fxyz[index] = coord

        return coord

    def StanceStep(self, phase, L, LateralFraction, YawRate, penetration_depth, T_bf, key, index):
        """
        Calcula las coordenadas del paso para el periodo sinusoidal STANCE
        usando una combinacion de coordenadas de paso lineal y rotacional
        inicialmente descompuesta de la entrada del usuario:
        L, LateralFraction y YawRate
        
        :param penetration_depth: Profundidad de penetracion del pie
                                  durante la fase de STANCE
        
        :return: Coordenadas del pie relativas al cuerpo
        """

        # Angulo del pie para movimiento tangente al circulo en Yaw
        phi_arc = self.YawCircle(T_bf, index)

        # Obtiene coordenadas del pie para movimiento lineal
        X_delta_lin, Y_delta_lin, Z_delta_lin = self.SineStance(
                phase, L, LateralFraction, penetration_depth
                )

        X_delta_rot, Y_delta_rot, Z_delta_rot = self.SineStance(
                phase, YawRate, phi_arc, penetration_depth
                )

        coord = np.array([
            X_delta_lin + X_delta_rot,
            Y_delta_lin + Y_delta_rot,
            Z_delta_lin + Z_delta_rot
            ])

        self.Pre_fxyz[index] = coord

        return coord

    def GetFootStep(self, L, LateralFraction, YawRate, cleareance_height,
                    penetration_depth, Tstance, T_bf, index, key):
        """
        Calcula las coordenadas del paso ya sea en la porcion de Bezeir o
        sinusoidad de la trayectoria, segun la fase obtenida
        """

        phase, StanceSwing = self.GetPhase(index, Tstance, self.Tswing)

        if StanceSwing == self.SWING:
            stored_phase = phase + 1.0
        else:
            stored_phase = phase

        # Solo para seguimiento
        self.Phase[index] = stored_phase

        if StanceSwing == self.STANCE:
            return self.StanceStep(phase, L, LateralFraction, YawRate, penetration_depth, T_bf, key, index)
        elif StanceSwing == self.SWING:
            return self.SwingStep(phase, L, LateralFraction, YawRate, cleareance_height, T_bf, key, index)

    def GenerateTrayectory(self,
                           L,
                           LateralFraction,
                           YawRate,
                           vel,
                           T_bf_,
                           T_bf_curr,
                           clearance_height = 0.06,
                           penetration_depth = 0.01,
                           contacts = [0, 0, 0, 0],
                           dt = None):

        """
        Calcula las coordenadas del paso para cada pata
        """
        # Primero, obtenemos Tstance a partir de la velocidad deseada y la longitud
        # la zancada

        # NOTA: L es la MITAD de la longitud de la zancada
        if vel != 0.0:
            Tstance = 2.0 * abs(L) / abs(vel)
        else:
            Tstance = 0.0
            L = 0.0
            self.TD = False
            self.time = 0.0
            self.time_since_last_TD = 0.0

        # Luego, obtenemos el tiempo desde el ultimo touchdown e Incrementaremos
        # el contador de Tiempo
        if dt is None:
            dt = self.dt

        YawRate *= dt

        # Detectamos pasos de tiempo inviables
        if Tstance < dt:
            Tstance = 0.0
            L = 0.0
            self.TD = False
            self.time = 0.0
            self.time_since_last_TD = 0.0
            YawRate = 0.0
        # NOTA: ES MUCHO MAS ESTABLE CON ESTO
        elif Tstance > 1.3 * self.Tswing:
            Tstance = 1.3 * self.Tswing

        # Verificamos contactos
        if contacts[0] == 1 and Tstance > dt:
            self.TD = True

        self.Increment(dt, Tstance + self.Tswing)

        T_bf = copy.deepcopy(T_bf_)

        for i, (key, Tbf_in) in enumerate(T_bf_.items()):
            # TODO: HACER ESTO MAS ELEGANTE
            if key == "FL":
                self.ref_idx = i
                self.dSref[i] = 0.0
            if key == "FR":
                self.dSref[i] = 0.5
            if key == "RL":
                self.dSref[i] = 0.5
            if key == "RR":
                self.dSref[i] = 0.0
            _, p_bf = TransToRp(Tbf_in)

            if Tstance > 0.0:
                step_coord = self.GetFootStep(L, LateralFraction, YawRate,
                                              clearance_height,
                                              penetration_depth,
                                              Tstance, p_bf, i, key)
            else:
                step_coord = np.array([0.0, 0.0, 0.0])

            T_bf[key][0, 3] = Tbf_in[0, 3] + step_coord[0]
            T_bf[key][1, 3] = Tbf_in[1, 3] + step_coord[1]
            T_bf[key][2, 3] = Tbf_in[2, 3] + step_coord[2]

        return T_bf

