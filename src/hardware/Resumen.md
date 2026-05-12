# Resumen de Cambios: Hardware Manager

He creado la clase `HardwareManager` basándome en tu script manual, integrándolo con tu sistema de configuración actual y eliminando los archivos obsoletos.

## ¿Qué se ha implementado?

1. **Clase `HardwareManager` limpia y estructurada:**
   - La nueva clase vive en `src/hardware/hardware_manager.py`.
   - Reemplaza de forma segura a `calibration_direction.py`, `Servo.py` y `ServoController.py`, eliminando la dependencia redundante de `adafruit_motor.servo`.

2. **Inicialización Dinámica (`_init_boards`):**
   - El código lee qué placas PCA9685 (1 y/o 2) están siendo usadas por los servos en tu `ServoRepository`.
   - Inicializa solo las placas necesarias con la dirección I2C y frecuencia correctas indicadas en tu `config.py`.

3. **Cálculo de PWM Personalizado (`set_angle`):**
   - Tal como querías, la función para asignar el ángulo utiliza el mismo cálculo manual de pulsos de tu script de calibración: 
     ```python
     pulse_range = servo.max_pulse - servo.min_pulse
     pulse_us = servo.min_pulse + (pulse_range * angle / 180.0)
     duty_cycle = int((pulse_us / 1_000_000.0) * pca.frequency * 0xFFFF)
     ```
   - Antes de aplicar la fórmula matemática, se aplica la lógica del repositorio: `invert_direction` y `offset`.
   - El ángulo resultante se limita entre 0 y 180 grados para seguridad de los servos físicos.

4. **Funciones de Conveniencia:**
   - `set_leg_angles(leg_id, angles)`: Toma un ID de pata (ej. "FL") y envía automáticamente una lista de ángulos a los servos correspondientes en base a `leg_map`.
   - `disable_all()`: Pone todos los *duty cycles* de todos los canales de todas las placas activas en `0`. Esto "apaga" los servos, dejándolos libres de fuerza.
   - `deinit()`: Apaga los servos y cierra las conexiones de I2C.

## ¿Cómo utilizarlo?

A partir de ahora, la forma de controlar el hardware en tu programa principal o en tus scripts de prueba es la siguiente:

```python
from src.utils.servo_repository import ServoRepository
from src.utils.toml_loader import TomlLoader
from src.hardware.hardware_manager import HardwareManager

# 1. Cargar configuración y repositorio
repo = ServoRepository()
loader = TomlLoader(repo)
loader.load_from_file()

# 2. Inicializar el administrador de hardware
hw = HardwareManager(loader, repo)

# 3. Mover servos!
hw.set_angle("FL_hip_roll", 90) # Mover un servo
hw.set_leg_angles("FR", [90, 45, -30]) # Mover una pata completa

# 4. Apagar / Liberar recursos
hw.deinit()
```

## Archivos Eliminados
Como aprobaste en el plan, he borrado de forma segura los siguientes archivos para mantener la carpeta `src/hardware` impecable:
- `src/hardware/calibration_direction.py`
- `src/hardware/Servo.py`
- `src/hardware/ServoController.py`
