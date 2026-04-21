import tomlkit
from pathlib import Path


class Config:
    def __init__(self):
        # Ruta a la carpeta de config
        self.config_path = Path.cwd() / "src" / "config"
        self._load_config()
    
    def _load_config(self):
        def load_toml(name):
            config_dir = self.config_path / name
            print(config_dir)
            with open(config_dir, 'r') as file:
                return tomlkit.load(file)
        self.robot_cfg = load_toml("robot.toml")
    
    @property
    def boards(self):
        return self.robot_cfg["motion_controller"]["boards"]
    
    @property
    def servos(self):
        return self.robot_cfg["motion_controller"]["servos"]
    
    @property
    def leg_map(self):
        lista = []
        for name in self.servos.keys():
            lista.append(name)
        
        leg_map = {
            "FL": lista[0:3],
            "FR": lista[3:6],
            "RL": lista[6:9],
            "RR": lista[-3:]
        }
        return leg_map
        

if __name__ == "__main__":
    c = Config()   
    print(c.leg_map)