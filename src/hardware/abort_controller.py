from gpiozero import LED
from src.utils.logger import log

class AbortController:
    def __init__(self):
        log.debug("Iniciando control relay")
        self.relay = LED(17, active_high=True)
        self.abort()
    
    def activate_servos(self):
        log.debug("Rele activado")
        self.relay.on()
    
    def abort(self):
        log.debug("Rele apagado")
        self.relay.off()
        
if __name__ == "__main__":
    import time
    led = AbortController()
    led.activate_servos()
    time.sleep(2)
    led.abort()