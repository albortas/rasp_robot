import tomllib
import tomli_w
from pathlib import Path
from typing import TYPE_CHECKING
from src.utils.servo import Servo

if TYPE_CHECKING:
    from src.utils.servo_repository import ServoRepository

class TomlLoader:
    """
    Se encarga de leer el archivo TOML, convertirlo a objetos Servo
    y cargarlos en el Repositorio. También sincroniza los cambios
    del repositorio de vuelta al archivo.
    """
    
    def __init__(self, repository: 'ServoRepository'):
        self.file_name = "robot.toml"
        self.file_path = Path(__file__).resolve().parent.parent / "config" / self.file_name
        self.repository = repository
        self._boards_data: dict = {} # Guarda la config de las placas

    @property
    def boards(self) -> dict:
        """Devuelve la configuracion de las placas PCA9685 cargadas desde el TOML."""
        return self._boards_data

    def load_from_file(self):
        """Lee el TOML y llena el repositorio con objetos Servo."""
        with open(self.file_path, "rb") as f:
            data = tomllib.load(f)

        self.repository.clear_servos()
        self._boards_data = data.get("boards", {})

        if "servos" in data:
            for key, value in data["servos"].items():
                servo_obj = Servo(name=key, **value)
                self.repository.insert_servo(servo_obj)

    def save_file(self):
        """Lee el repositorio y sobrescribe el archivo TOML."""
        data_save = {"boards": self._boards_data, "servos": {}}

        for servo in self.repository.get_servos():
            data_save["servos"][servo.name] = servo.to_dict()

        if tomli_w is not None:
            with open(self.file_path, "wb") as f:
                tomli_w.dump(data_save, f)
        else:
            with open(self.file_path, "w", encoding="utf-8") as f:
                f.write(tomllib.dumps(data_save))

    def synchronize(self):
        """Atajo: guarda el estado actual del repositorio en el archivo."""
        self.save_file()

if __name__ == "__main__":
    from src.utils.servo_repository import ServoRepository
    repo = ServoRepository()
    loader = TomlLoader(repo)

    loader.load_from_file()
    print(repo.get_servo_names())
