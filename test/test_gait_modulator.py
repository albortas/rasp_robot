import unittest
import sys

# Ajusta según el nombre de tu módulo
try:
    from src.gait.bezier_gait import BazierGait
except ImportError:
    print("ERROR: No se encuentra el módulo 'gait_controller'.")
    print("Cambia la línea 'from ... import BazierGait'")
    sys.exit(1)


class TestBazierGaitAgainstPaper(unittest.TestCase):
    """
    Tests unitarios para validar que la implementación de BazierGait
    coincide con las Ecuaciones (13) y (15) del paper del MIT Cheetah 2014.
    """

    def setUp(self):
        self.Tswing = 0.25
        self.Tstance = 0.5
        self.Tstride = self.Tstance + self.Tswing  # 0.75
        self.gait = BazierGait(
            dSref=[0.0, 0.5, 0.5, 0.0],
            Tswing=self.Tswing
        )
        self.gait.ref_idx = 0

    def test_equation_13_get_ti(self):
        self.gait.time_since_last_TD = 0.3
        ti_FL = self.gait.Get_ti(0, self.Tstride)
        self.assertAlmostEqual(ti_FL, 0.3, places=6)
        ti_FR = self.gait.Get_ti(1, self.Tstride)
        expected_ti_FR = 0.3 - 0.5 * self.Tstride
        self.assertAlmostEqual(ti_FR, expected_ti_FR, places=6)

    def test_equation_15_stance_phase(self):
        self.gait.time_since_last_TD = 0.25
        phase, state = self.gait.GetPhase(0, self.Tstance, self.Tswing)
        self.assertEqual(state, self.gait.STANCE)
        self.assertAlmostEqual(phase, 0.25 / self.Tstance, places=6)

    def test_equation_15_swing_negative_zone(self):
        self.gait.time_since_last_TD = 0.2
        phase, state = self.gait.GetPhase(1, self.Tstance, self.Tswing)
        self.assertEqual(state, self.gait.SWING)
        expected_phase = (-0.175 + self.Tswing) / self.Tswing
        self.assertAlmostEqual(phase, expected_phase, places=6)

    def test_equation_15_swing_positive_zone(self):
        self.gait.time_since_last_TD = 0.65
        phase, state = self.gait.GetPhase(0, self.Tstance, self.Tswing)
        self.assertEqual(state, self.gait.SWING)
        expected_phase = (0.65 - self.Tstance) / self.Tswing
        self.assertAlmostEqual(phase, expected_phase, places=6)

    def test_wrap_around_logic(self):
        self.gait.time_since_last_TD = 0.1
        phase, state = self.gait.GetPhase(1, self.Tstance, self.Tswing)
        self.assertEqual(state, self.gait.STANCE)
        self.assertAlmostEqual(phase, 0.475 / self.Tstance, places=6)

    def test_touchdown_detection_sequence(self):
        # 1. Simulamos que la pata de referencia está al final del swing
        self.gait.time_since_last_TD = self.Tstride  # ti = 0.75
        phase, state = self.gait.GetPhase(0, self.Tstance, self.Tswing)
        self.assertAlmostEqual(phase, 1.0, places=6)
        self.assertTrue(self.gait.TD)

        # 2. Guardamos el tiempo actual (antes de incrementar)
        tiempo_antes = self.gait.time  # será 0.0

        dt = 0.001
        self.gait.Increment(dt, self.Tstride)

        # CheckTouchDown se ejecuta con self.time == tiempo_antes,
        # por lo que TD_time debe ser igual a tiempo_antes.
        self.assertAlmostEqual(self.gait.TD_time, tiempo_antes, places=3,
                               msg="CheckTouchDown debe guardar el tiempo actual (antes del incremento)")
        self.assertFalse(self.gait.TD)
        self.assertAlmostEqual(self.gait.SwRef, 0.0, places=3)


if __name__ == '__main__':
    unittest.main(argv=['first-arg-is-ignored'], exit=False)