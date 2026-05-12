import json
from pathlib import Path

# Dependencias para TOML (igual que antes)
try:
    import tomllib
except ImportError:
    try:
        import toml as tomllib
    except ImportError:
        raise SystemExit("Instalá la dependencia con: pip install toml")

try:
    import tomli_w
except ImportError:
    tomli_w = None

# ═══════════════════════════════════════════════════════════════
# 1. CLASE SERVO (El objeto de datos)
# ═══════════════════════════════════════════════════════════════
class Servo:
    """Representa un solo servo con todos sus parámetros de calibración."""

    def __init__(self, nombre: str, pca9685: int, channel: int, 
                 min_pulse: int, max_pulse: int, rest_angle: float, 
                 invert_direction: bool, offset: int):
        
        self.nombre = nombre
        self.pca9685 = pca9685
        self.channel = channel
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.rest_angle = rest_angle
        self.invert_direction = invert_direction
        self.offset = offset

    def actualizar(self, **kwargs):
        """Modifica atributos del servo dinámicamente."""
        for clave, valor in kwargs.items():
            if hasattr(self, clave):
                setattr(self, clave, valor)
            else:
                raise AttributeError(f"El servo no tiene el atributo '{clave}'")

    def to_dict(self) -> dict:
        """Convierte el objeto a un diccionario plano para guardar en TOML."""
        return {clave: valor for clave, valor in self.__dict__.items() 
                if clave != 'nombre'}

    def __repr__(self):
        dir_str = "INV" if self.invert_direction else "NOR"
        return f"<Servo {self.nombre} | CH:{self.channel} | PCA:{self.pca9685} | {dir_str}>"


# ═══════════════════════════════════════════════════════════════
# 2. REPOSITORIO DE SERVOS (Lógica de memoria)
# ═══════════════════════════════════════════════════════════════
class RepositorioServos:
    """Maneja la colección de objetos Servo en memoria."""

    def __init__(self):
        self._servos: dict[str, Servo] = {}

    def agregar(self, servo: Servo):
        """Añade un servo nuevo a la colección."""
        self._servos[servo.nombre] = servo

    def seleccionar(self, nombre: str) -> Servo:
        """Retorna un objeto Servo por su nombre. Lanza error si no existe."""
        if nombre not in self._servos:
            raise KeyError(f"Servo '{nombre}' no encontrado.")
        return self._servos[nombre]

    def modificar(self, nombre: str, **kwargs) -> Servo:
        """Modifica atributos de un servo existente y retorna el objeto actualizado."""
        servo = self.seleccionar(nombre)
        servo.actualizar(**kwargs)
        return servo

    def eliminar(self, nombre: str):
        """Elimina un servo de la colección."""
        if nombre not in self._servos:
            raise KeyError(f"Servo '{nombre}' no encontrado para eliminar.")
        del self._servos[nombre]

    def obtener_todos(self) -> list[Servo]:
        """Retorna una lista con todos los objetos Servo."""
        return list(self._servos.values())

    def obtener_por_pata(self, pata: str) -> list[Servo]:
        """Filtra servos por el prefijo de la pata (ej: 'FL')."""
        return [s for s in self.obtener_todos() if s.nombre.startswith(f"{pata.upper()}_")]

    def obtener_por_board(self, board_id: int) -> list[Servo]:
        """Filtra servos conectados a un PCA9685 específico."""
        return [s for s in self.obtener_todos() if s.pca9685 == board_id]

    def limpiar(self):
        """Vacía la colección completa."""
        self._servos.clear()


# ═══════════════════════════════════════════════════════════════
# 3. CARGADOR TOML (Puente entre el archivo y el repositorio)
# ═══════════════════════════════════════════════════════════════
class CargadorTOML:
    """
    Se encarga de leer el archivo TOML, convertirlo a objetos Servo
    y cargarlos en el Repositorio. También sincroniza los cambios
    del repositorio de vuelta al archivo.
    """
    
    def __init__(self, ruta_archivo: str, repositorio: RepositorioServos):
        self.ruta = Path(ruta_archivo)
        self.repositorio = repositorio
        self._boards_data: dict = {} # Guarda la config de las placas

    def cargar_desde_archivo(self):
        """Lee el TOML y llena el repositorio con objetos Servo."""
        with open(self.ruta, "rb") as f:
            datos = tomllib.load(f)

        self.repositorio.limpiar()
        self._boards_data = datos.get("boards", {})

        if "servos" in datos:
            for nombre, params in datos["servos"].items():
                servo_obj = Servo(nombre=nombre, **params)
                self.repositorio.agregar(servo_obj)

    def guardar_en_archivo(self):
        """Lee el repositorio y sobrescribe el archivo TOML."""
        datos_para_guardar = {"boards": self._boards_data, "servos": {}}

        for servo in self.repositorio.obtener_todos():
            datos_para_guardar["servos"][servo.nombre] = servo.to_dict()

        if tomli_w is not None:
            with open(self.ruta, "wb") as f:
                tomli_w.dump(datos_para_guardar, f)
        else:
            with open(self.ruta, "w", encoding="utf-8") as f:
                f.write(tomllib.dumps(datos_para_guardar))

    def sincronizar(self):
        """Atajo: guarda el estado actual del repositorio en el archivo."""
        self.guardar_en_archivo()


# ═══════════════════════════════════════════════════════════════
# EJEMPLO DE USO (Flujo completo)
# ═══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    
    ARCHIVO = "servos_config.toml"

    # (Opcional) Crear el archivo de prueba si no existe
    if not Path(ARCHIVO).exists():
        with open(ARCHIVO, "w") as f:
            f.write("""[boards.pca9685_1]\naddress = "0x40"\nfrequency = 50\n\n[boards.pca9685_2]\naddress = "0x42"\nfrequency = 50\n\n[servos.FL_hip_roll]\npca9685 = 1\nchannel = 0\nmin_pulse = 500\nmax_pulse = 2600\nrest_angle = 90.0\ninvert_direction = false\noffset = 0\n\n[servos.FL_hip_pitch]\npca9685 = 1\nchannel = 1\nmin_pulse = 500\nmax_pulse = 2650\nrest_angle = 90.0\ninvert_direction = false\noffset = 0\n""")

    # 1. Inicializar el sistema
    repo = RepositorioServos()
    cargador = CargadorTOML(ARCHIVO, repo)
    
    # 2. Cargar datos desde el TOML a los objetos en memoria
    cargador.cargar_desde_archivo()
    print("✅ Archivo cargado en memoria.\n")

    # ── SELECCIONAR ────────────────────────────────────────────
    print("--- SELECCIONAR ---")
    mi_servo = repo.seleccionar("FL_hip_roll")
    print(f"Objeto obtenido: {mi_servo}")
    print(f"Su pulso máximo es: {mi_servo.max_pulse}\n")

    # ── MODIFICAR (en memoria) ────────────────────────────────
    print("--- MODIFICAR ---")
    repo.modificar("FL_hip_roll", channel=14, offset=15, invert_direction=True)
    print(f"Después de modificar: {mi_servo}\n") # El objeto se actualizó solo
    
    # Sincronizamos con el archivo TOML
    cargador.sincronizar()
    print("💾 Archivo TOML actualizado con los cambios.\n")

    # ── AÑADIR (en memoria y archivo) ─────────────────────────
    print("--- AÑADIR ---")
    nuevo_servo = Servo(
        nombre="FL_knee", pca9685=1, channel=2,
        min_pulse=500, max_pulse=2500, rest_angle=90.0,
        invert_direction=False, offset=0
    )
    repo.agregar(nuevo_servo)
    cargador.sincronizar()
    print(f"Servo añadido: {nuevo_servo}\n")

    # ── FILTRAR / SELECCIONAR MÚLTIPLES ───────────────────────
    print("--- FILTRAR POR PATAs ---")
    servos_fl = repo.obtener_por_pata("FL")
    for s in servos_fl:
        print(f" - {s}")

    # ── ELIMINAR (en memoria y archivo) ───────────────────────
    print("\n--- ELIMINAR ---")
    repo.eliminar("FL_hip_pitch")
    cargador.sincronizar()
    print("Servo 'FL_hip_pitch' eliminado. Archivo sincronizado.\n")