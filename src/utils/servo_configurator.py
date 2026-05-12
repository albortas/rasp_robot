from src.utils.servo_repository import ServoRepository
from src.utils.toml_loader import TomlLoader

class ServoConfigurator:
    def __init__(self, repository=None, loader=None):
        if repository is None or loader is None:
            self._repository = ServoRepository()
            self.toml_loader = TomlLoader(self._repository)
            self.toml_loader.load_from_file()
        else:
            self._repository = repository
            self.toml_loader = loader
        self.servo_names = self._repository.get_servo_names()

    def update_channels(self, number_pcas = 1):
        if number_pcas == 1:
            channel_disable = [3, 7, 11]
            channel_enable = [channel for channel in range(15) if channel not in channel_disable]
            for name, channel in zip(self.servo_names, channel_enable):
                self._repository.update_servo(name, pca9685= 1, channel= channel)
        elif number_pcas == 2:
            channel_enable = [ch for ch in range(7) if ch != 3]
            for name, channel in zip(self.servo_names[0:6], channel_enable):
                self._repository.update_servo(name, pca9685= 1, channel= channel)
            for name, channel in zip(self.servo_names[6:], channel_enable):
                self._repository.update_servo(name, pca9685= 2, channel= channel)
        else:
            print("Error: Solo se pueden configurar 1 o 2")
        self.toml_loader.synchronize()

    def update_offset(self, name, offset= 0):
        servo = self._repository.update_servo(name, offset= offset)
        print(f"Servo {servo.name} offset actulizado a {servo.offset}")
    
    def update_invert_direction(self, name, invert_direction= False):
        servo = self._repository.update_servo(name, invert_direction= invert_direction)
        print(f"Servo {servo.name} direccion actualizada a {servo.invert_direction}")
    
if __name__ == "__main__":
    configurator = ServoConfigurator()
