# Resumen: Menú Interactivo de Calibración

He terminado la implementación del menú de calibración interactivo tomando en cuenta todas tus indicaciones. El nuevo script se llama `calibrate_servo.py` y vive en la carpeta `calibration/`.

## ¿Qué se ha implementado?

1. **Nuevo Script Interactivo (`calibrate_servo.py`)**
   - Utiliza la librería `pick` para mantener la interfaz visual y elegante que te gusta en la terminal.
   - Cuenta con "Modo Simulación" automático si no detecta las placas PCA (útil para cuando trabajas desde el PC).
   - Fuerza a que todos los servos usen `pca_1` al iniciarse.

2. **Flujo de Calibración Unificado**
   - El menú ahora usa **la misma memoria** para `HardwareManager` y `ServoConfigurator`. Esto significa que al modificar un offset en el submenú de "Ajuste Fino", se guarda en el archivo TOML y automáticamente se envía el nuevo ángulo al servo en tiempo real.
   - Mismo caso para el cambio de inversión ("Invertir Dirección"), el cambio es instantáneo.

3. **Postura Segura (`rest_angle`)**
   - En `HardwareManager`, he añadido la función `go_to_rest_pose()` que **se ejecuta automáticamente** cada vez que instancias la clase. A partir de ahora, cuando arranques el hardware, todos los servos irán a su posición de descanso.
   - En el menú de calibración, cada vez que aplicas un ajuste de offset, el servo reevalúa su `rest_angle` para que puedas observar físicamente el cambio.

4. **Limpieza**
   - `config_servo.py` ha sido borrado exitosamente y reemplazado de manera íntegra.
   - `ServoConfigurator` ahora puede recibir el repositorio por inyección de dependencias de manera limpia.

## ¿Cómo ejecutarlo?

Simplemente abre una terminal en la raíz de tu proyecto y ejecuta:

```bash
python calibration/calibrate_servo.py
```

Si estás en la Raspberry Pi, los servos responderán a tus selecciones. Si estás en tu PC local, te avisará que está en "Modo Simulación" y podrás navegar por la lógica y guardar los offsets en el TOML.
