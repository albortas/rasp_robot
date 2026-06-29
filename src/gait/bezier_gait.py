
class BazierGait():
    # Estos binarios
    STANCE = 0  # Contacto con el suelo
    SWING = 1   # Pierna en el aire
    def __init__(self, dSref = [0.0, 0.0, 0.5, 0.5], Tswing = 0.2) -> None:
        self.dSref = dSref  # Desfase de fase por pata: [FL, FR, RL, RR]
        # La pata de referencia es FL, siempre 0
        self.ref_idx = 0.0  # Pata de referencia
        self.time_since_last_TD = 0.0 # Tiempo transcurrido desde el último touchdown
        self.StanceSwing = self.STANCE  # Modo de la trayectoria
        self.SwRef = 0.0  # Valor de fase en la fase de swing [0, 1] de la pata de referencia
        self.TD = False # Indica si la pata de referencia ha tocado el suelo
        self.TD_time = 0.0  # Tiempo de touchdown
        self.time = 0.0 # Tiempo total transcurrido
        self.Tswing = Tswing    # Tiempo de SWING

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

            # Contacto con el suelo de la pierna de referencia al final del SWING
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

        # Incrementaremos el tiempo al final, en caso de que acabe de ocurrir un touchdown
        self.time += dt

        # Si Tstride = Tswing, entonces Tstance = 0
        # REINICIAMOS TODO
        if Tstride < self.Tswing + dt:
            self.time = 0.0
            self.time_since_last_TD = 0.0
            self.TD_time = 0.0
            self.SwRef = 0.0

