# AGENTS.md — Robodelta

Robot cuadrúpedo con Python 3.12+. Código fuente en `src/`. Comentarios, logs y
UI en español; nombres de variables/clases/funciones en inglés.

## Instalación

```bash
pip install -e ".[pc]"    # desarrollo en PC (sin hardware)
pip install -e ".[rpi]"   # Raspberry Pi (con Adafruit PCA9685)
```

No hay entry points definidos en pyproject.toml. Ejecutar con `python src/main.py`.

## Entorno dual (PC vs RPi)

- `HardwareManager` importa `adafruit_pca9685`, `board`, `busio` a nivel de módulo
  — **sin try/except**. Falla al importar en PC.
- `ServosPowerController` importa `gpiozero.LED` a nivel de módulo — mismo problema.
- `main.py` importa ambos incondicionalmente → **no se puede ejecutar `main.py`
  en PC** sin instalar `[rpi]` o mockear hardware.
- `calibrate_servo.py` sí detecta el entorno con try/except y cae a modo simulación.
- `HardwareManager.__init__` ejecuta `go_to_rest_pose()` automáticamente al construirse.

## Arquitectura

```
src/main.py                        → punto de entrada, clase Robot con RobotMode enum
src/control/control_interface.py   → capa de integración cinemática ↔ hardware
src/control/gait_inteface.py       → interfaz de marcha (typo en nombre, sin 'r')
src/controller/PS4Controller.py    → mando PS4 (pygame)
src/hardware/hardware_manager.py   → control PCA9685 (solo RPi)
src/hardware/servos_power_controller.py → relay GPIO pin 17 (gpiozero, solo RPi)
src/gait/bezier_gait.py            → generador de marcha curvas Bézier
src/kinematics/                    → cinemática inversa/directa, modelo del robot
src/utils/                         → Servo, ServoRepository, TomlLoader, logs
calibration/calibrate_servo.py     → calibración interactiva (pick)
test/                              → 1 unittest real + 3 scripts interactivos matplotlib
```

## Modos de operación (main.py)

`RobotMode(IntEnum)`: NEUTRAL(0), STATIC_POSTURE(1), GAIT_MODE(2).
Botones PS4: Options(9)=toggle potencia, L0(4)=postura estática, R0(5)=marcha.
Loop principal a 50Hz (`LOOP_PERIOD = 0.02s`).

## Errores conocidos en nombres

Estos typos están en el código fuente y propagados en imports — no corregir sin
actualizar todos los referencers:

- `gait_inteface.py` / `GaitInteface` → falta 'r' (debería ser `gait_interface`)
- `BazierGait` → falta 'e' (debería ser `BezierGait`)

## Configuración

- **Canónica**: `src/config/robot.toml` — la que carga `TomlLoader`.
- `TomlLoader.__init__` auto-carga `robot.toml` al instanciarse.
- Los demás TOML en `src/config/` son legacy (`config.toml`, `config_servos.toml`,
  `robot copy.toml`).

## Nomenclatura de servos

Patrón `{LEGA}_{ARTICULACION}`. 12 servos, 3 por pata:

- Patas: `FL`, `FR`, `RL`, `RR` (Front-Left, Front-Right, Rear-Left, Rear-Right)
- Articulaciones: `hip_roll`, `hip_pitch`, `knee`
- Ejemplo: `FL_hip_roll`, `RR_knee`
- Dos placas PCA9685: FL+FR en placa 1 (0x40), RL+RR en placa 2 (0x41)

## Canales PCA9685 omitidos

Los canales 3, 7, 11 y 15 (en modo 1 placa) están deshabilitados intencionalmente.

## Tests

- `test/test_gait_modulator.py` — unittest.TestCase con 6 tests (el único test real).
- Los demás archivos en `test/` son scripts interactivos con matplotlib sliders.
- No hay CI, no hay `Makefile`, no hay pre-commit hooks.

## Linter

Ruff se usó ad-hoc desde CLI pero no hay configuración (ni `ruff.toml` ni
`[tool.ruff]` en pyproject.toml).

## Patrón TYPE_CHECKING

Varios módulos (`hardware_manager.py`, `channels_configurator.py`, `toml_loader.py`,
`servo_repository.py`) usan `if TYPE_CHECKING:` para evitar imports circulares.

## egg-info desactualizado

`Robot.egg-info/` referencia archivos eliminados (`ServoController.py`,
`ControlInterface.py`, `LieAlgebra.py`, etc.). Regenerar con `pip install -e .`
si se cambia la estructura del paquete.
