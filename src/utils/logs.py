import logging
from pathlib import Path

LOGGER_NAME = "robot"
# Verifica si ya fue configurado (patron singleton)
_is_config = False


def setup_logging():
    """Configurar logging para todo el robot"""
    global _is_config

    # Crear un logger especifico (no usar root)
    logger = logging.getLogger(LOGGER_NAME)

    # Configuracion de Handler. Lo ignoramos
    if _is_config:
        return logger

    # --- Configuracion ---
    logger.setLevel(logging.DEBUG)

    # Formato detallado para archivos
    detailed = logging.Formatter(
        "%(asctime)s | %(name)s | %(levelname)-8s | "
        "%(filename)s:%(lineno)d | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    # Formato simple para consola
    simple = logging.Formatter("%(levelname)-8s | %(message)s")

    # --- Handler 1: Archivo (TODO, incluido DEBUG) ---
    file_log = Path(LOGGER_NAME + ".log")
    file_handler = logging.FileHandler(file_log, mode="w")
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(detailed)

    # --- Handler 2: Consola (solo INFO o superior) ---
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(simple)

    # Asociar ambos handler al logger
    logger.addHandler(console_handler)
    logger.addHandler(file_handler)

    # Evitar propagación al root logger (causa duplicacion)
    logger.propagate = False

    # Marcar como configurado
    _is_config = True

    return logger


# Logger por defecto para importar directamete
log = setup_logging()
