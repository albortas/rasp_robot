# src/utils/servo.py
from dataclasses import dataclass, asdict

@dataclass
class Servo:
    """Representación de un servo del robot, optimizada para PC y RPi."""
    name: str
    pca9685: int
    channel: int
    min_pulse: int
    max_pulse: int
    rest_angle: int
    offset: int
    invert_direction: bool

    def update_servo(self, **kwargs):
        """Modifica atributos dinámicamente"""
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
            else:
                raise KeyError(f'El servo no tiene el atributo: {key}')

    def to_dict(self) -> dict:
        """Convierte el objeto a un diccionario plano para guardar en archivo TOML"""
        # Excluir 'name'
        return {key: value for key, value in asdict(self).items() if key != "name"}

if __name__ == "__main__":
    servo_1 = Servo(
        name="FL_knee",
        pca9685=1,
        channel=0,
        min_pulse=500,
        max_pulse=2500,
        rest_angle=90,
        offset=-4,
        invert_direction=False
    )

    # Actualizar atributos dinámicamente
    servo_1.update_servo(name="FL_knee", channel=4, offset=-8)

    # El __repr__ será automático y el __eq__ también
    print(servo_1)
    print(servo_1.to_dict())
