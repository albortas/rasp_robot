from gpiozero import LED
from src.utils.logger import log

class ServosPowerController:
    def __init__(self):
        log.debug("Iniciando control relay")
        self.relay = LED(17, active_high=True)
        self.state = False
        self.off()
    
    def on(self):
        self.state = True
        log.info("Rele activado")
        self.relay.on()
    
    def off(self):
        self.state = False
        log.info("Rele apagado")
        self.relay.off()
    
    def toggle(self):
        if self.state:
            self.off()
        else:
            self.on()
        
if __name__ == "__main__":
    import time
    led = ServosPowerController()
    try:
        while True:
            led.toggle()
            time.sleep(1)
    except KeyboardInterrupt:
        print("Terminado por el usurio")
    finally:
        led.abort()
        print("Saliendo..")