from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.utils.servo import Servo

class ServoRepository:
    """Maneja la coleccion de objetos Servos en memoria"""
    def __init__(self):
        self._servos: dict[str, 'Servo'] = {}
    
    def insert_servo(self, servo: 'Servo'):
        """Aniade un nuevo Servo a la coleccion"""
        self._servos[servo.name] = servo

    def select_servo(self, name: str) -> 'Servo':
        """Retorna el objeto Servo por su nombre."""
        if name not in self._servos:
            raise KeyError(f'Servo {name} no encontrado.')
        return self._servos[name]
    
    def update_servo(self, name: str, **kwargs) -> 'Servo':
        """Modifica atributos de un Servo existente y devuelve el objeto actualizado"""
        servo = self.select_servo(name)
        servo.update_servo(**kwargs)
        return servo

    def delete_servo(self, name: str):
        """Elimina un servo de la colección."""
        if name not in self._servos:
            raise KeyError(f"Servo '{name}' no encontrado para eliminar.")
        del self._servos[name]

    def get_servo_names(self) -> list[str]:
        """Retorna una lista con los nombres de todos los objetos Servos"""
        return list(self._servos.keys())

    def get_servos(self) -> list['Servo']:
        """Retorna una lista con todos los objetos Servo."""
        return list(self._servos.values())

    def get_by_leg(self, leg: str) -> list['Servo']:
        """Filtra servos por el prefijo de la pata (ej: 'FL')."""
        return [servo for servo in self.get_servos() if servo.name.startswith(f"{leg.upper()}_")]

    def get_by_board(self, board_id: int) -> list['Servo']:
        """Filtra servos conectados a un PCA9685 específico."""
        return [servo for servo in self.get_servos() if servo.pca9685 == board_id]

    def clear_servos(self):
        """Vacía la colección completa."""
        self._servos.clear()
