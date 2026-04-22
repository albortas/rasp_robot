import tomlkit
from pathlib import Path

class FileConfigServos:
    def __init__(self):
        self.name_file = "config_servos.toml"
        self.file_path = Path.cwd() / "src/config" / self.name_file
        self._file_create()
    
    @property
    def joints_robot(self):
        return [
            "FL_hip_roll", "FL_hip_pitch", "FL_knee",
            "FR_hip_roll", "FR_hip_pitch", "FR_knee",
            "RL_hip_roll", "RL_hip_pitch", "RL_knee",
            "RR_hip_roll", "RR_hip_pitch", "RR_knee"
        ]

    def _config_servos(self):
        doc = tomlkit.document()
        for name_joint in self.joints_robot:
            doc[name_joint]={
                'min_pulse': 500,
                'max_pulse': 2500,
                'rest_angle': 90,
                'invert_direction': False,
                'offset': 0
            }
        return doc
             
    def _file_create(self):
        doc = self._config_servos()
        with open(self.file_path, "w") as file:
            tomlkit.dump(doc, file)
        print("Creado el archivo de configuracion para los servos")

if __name__ == "__main__":
    fcs = FileConfigServos()