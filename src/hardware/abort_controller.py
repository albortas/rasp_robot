from gpiozero import LED
from src.utils.logger import log

class AbortController:
    def __init__(self):
        log.debug("Iniciando control relay")
        self.relay = LED(17, active_high=True)
        self._state = False
        self.abort()
    
    def activate_servos(self):
        self._state = True
        log.debug("Rele activado")
        self.relay.on()
    
    def abort(self):
        self._state = False
        log.debug("Rele apagado")
        self.relay.off()
    
    def toggle(self):
        if self._state:
            self.abort()
        else:
            self.activate_servos()
        
if __name__ == "__main__":
    import time
    led = AbortController()
    try:
        while True:
            led.toggle()
            time.sleep(1)
    except KeyboardInterrupt:
        print("Terminado por el usurio")
    finally:
        led.abort()
        print("Saliendo..")