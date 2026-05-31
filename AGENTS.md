# AGENTS.md — Robodelta

Robot cuadrúpedo con Python 3.12+. Código fuente en `src/`. Comentarios, logs y
UI en español; nombres de variables/clases/funciones en inglés.

## Instalación

```bash
pip install -e ".[pc]"    # desarrollo en PC (sin hardware)
pip install -e ".[rpi]"   # Raspberry Pi (con Adafruit PCA9685)
```

## Entorno dual (PC vs RPi)

- `HardwareManager` importa `adafruit_pca9685`, `board`, `busio` — **solo funciona
  en RPi**. En PC lanza `ImportError`.
- `calibrate_servo.py` detecta esto automáticamente y cae a modo simulación.
- No asumir que los imports de hardware funcionan en PC.

## Dependencias

- `pyproject.toml` declara `tomllib`
  (stdlib) para lectura y `tomli_w` para escritura.

## Arquitectura

```
src/control/ControlInterface   → capa de integración (ROTA)
src/controller/PS4Controller   → mando PS4 (pygame)
src/hardware/hardware_manager  → control PCA9685 (solo RPi)
src/kinematics/                → cinemática inversa, modelo del robot
src/utils/                     → Servo, ServoRepository, TomlLoader, logs
calibration/calibrate_servo    → calibración interactiva (pick)
```

**Import roto**: `ControlInterface.py:4` importa `from src.hardware.ServoController
import ServoController` — ese archivo fue eliminado en `8c950e8`. Cualquier trabajo
en `ControlInterface` debe usar `HardwareManager`.

## Efectos secundarios al instanciar

- `TomlLoader(repo)` → auto-carga `src/config/robot.toml`
- `HardwareManager(loader, repo)` → auto-ejecuta `go_to_rest_pose()` (mueve todos
  los servos a su ángulo de reposo al construirse)

## Configuración

- **Canónica**: `src/config/robot.toml` — el que carga `TomlLoader`.
- Los demás TOML en `src/config/` son alternativos o legacy.

## Nomenclatura de servos

Patrón `{LEGA}_{ARTICULACION}`. 12 servos, 3 por pata:

- Patas: `FL`, `FR`, `RL`, `RR` (Front-Left, Front-Right, Rear-Left, Rear-Right)
- Articulaciones: `hip_roll`, `hip_pitch`, `knee`
- Ejemplo: `FL_hip_roll`, `RR_knee`

## Canales PCA9685 omitidos

Los canales 3, 7, 11 y 15 (en modo 1 placa) están deshabilitados intencionalmente.

## Sin tests, sin CI, sin linter configurado

No hay tests automatizados, solo bloques `if __name__ == "__main__"` para pruebas
manuales. Ruff se usó ad-hoc desde CLI pero no hay configuración.

## egg-info desactualizado

`Robot.egg-info/` referencia archivos que ya no existen. Regenerar con
`pip install -e .` si se cambia la estructura del paquete.

## Patrón TYPE_CHECKING

Varios módulos (`hardware_manager.py`, `channels_configurator.py`, `toml_loader.py`,
`servo_repository.py`) usan `if TYPE_CHECKING:` para evitar imports circulares.
