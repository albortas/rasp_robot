class Servo:
    def __init__(
        self,
        name: str,
        pca9685: int,
        channel: int,
        min_pulse: int,
        max_pulse: int,
        rest_angle: int,
        offset: int,
        invert_direction: bool,
    ):
        self.name = name
        self.pca9685 = pca9685
        self.channel = channel
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.rest_angle = rest_angle
        self.offset = offset
        self.invert_direction = invert_direction
        
    def update_servo(self, **kwargs):
        """Modifica atributos dinamicamente"""
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
            else:
                raise KeyError(f'El servo no tiene el atributo: {key}')

    def to_dict(self) -> dict:
        """Convierte el objeto a un diccionario plano para guardar en archivo TOML"""
        return {key: value for key, value in self.__dict__.items() if key != "name" }
    
    def __repr__(self):
        return f'{self.name} | pca: {self.pca9685} | ch: {self.channel}'

if __name__ == "__main__":
    servo_1 = Servo("FL_knee", 1, 0, 500, 2500, 90, -4, False)
    servo_1.update_servo(name = "FL_knee", channel = 4, offset= -8)
    print(servo_1.to_dict())
