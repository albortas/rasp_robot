import logging
from pathlib import Path


class Log:
    _logger = None
    _configured = False

    @classmethod
    def get(cls, name="robot"):
        if not cls._configured:
            cls._setup(name)
        return cls._logger

    @classmethod
    def _setup(cls, name):
        # Crear un logger especifico (no usar root)
        cls._logger = logging.getLogger(name)

        # --- Configuracion ---
        cls._logger.setLevel(logging.DEBUG)

        # Formato detallado para archivos
        detailed = logging.Formatter(
            "%(asctime)s | %(name)s | %(levelname)-8s | "
            "%(filename)s:%(lineno)d | %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )

        # Formato simple para consola
        simple = logging.Formatter("%(levelname)-8s | %(message)s")

        # --- Handler 1: Archivo (TODO, incluido DEBUG) ---
        file_log = Path(name + ".log")
        file_handler = logging.FileHandler(file_log, mode="w")
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(detailed)

        # --- Handler 2: Consola (solo INFO o superior) ---
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(simple)

        # Asociar ambos handler al logger
        cls._logger.addHandler(console_handler)
        cls._logger.addHandler(file_handler)

        # Evitar propagación al root logger (causa duplicacion)
        cls._logger.propagate = False

        # Marcar como configurado
        cls._configured = True

# Logger por defecto para importar directamete
log = Log.get()
