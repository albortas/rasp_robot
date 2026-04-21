import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class ServoControl:
    def __init__(self,
                 boards):
        self.boards = boards
        
    def _create_pca9685(self, i2c, board_config):
        pca = self.PCA9685(i2c, address=int)

    
    def _inil_pca9685(self):
        pass
