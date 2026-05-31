from typing import TYPE_CHECKING
from src.utils.logger import log

if TYPE_CHECKING:
    from src.utils.servo_repository import ServoRepository


class ChannelsConfigurator:
    """Configura canales, offsets y direccion de servos usando un repositorio y cargando TOML"""

    # Constantes de clase para canales deshabilitados
    _DISABLE_CHANNELS_SINGLE = frozenset({3, 7, 11, 15})
    _DISABLE_CHANNELS_DUAL = frozenset({3, 7})
    _CHANNELS_BY_PCA = 16
    _CHANNELS_BY_DUAL_PCA = 8
    _SERVOS_BY_DUAL_PCA = 6

    def __init__(self, repository: "ServoRepository"):
        self._repository = repository
        self._servo_names = self._repository.get_servo_names()

    @property
    def servo_names(self) -> list[str]:
        """Nombre de los servos configurados"""
        return self._servo_names.copy()

    @classmethod
    def _enable_channels_single(cls) -> list[int]:
        """Genera una lista de canales habilitados para una placa PCA9685"""
        return [
            channel
            for channel in range(cls._CHANNELS_BY_PCA)
            if channel not in cls._DISABLE_CHANNELS_SINGLE
        ]

    @classmethod
    def _enable_channels_dual(cls) -> list[int]:
        channels_dual = [
            channel
            for channel in range(cls._CHANNELS_BY_DUAL_PCA)
            if channel not in cls._DISABLE_CHANNELS_DUAL
        ]
        return channels_dual + channels_dual

    def update_channels(self, number_pcas: int = 1) -> None:
        """
        Asigna canales PCA9685 a los servos segun el numero de placas.

        Args:
            number_pcas: 1 o 2 placas PCA9685

        Raises:
            ValueError: Si number_pcas no es 1 ni 2
        """
        if number_pcas not in (1, 2):
            raise ValueError(
                f"Solo se puede configurar 1 o 2 PCAs. Recibido: {number_pcas}"
            )
        servos = self._servo_names
        
        if number_pcas == 1:
            channels = self._enable_channels_single()
            pca_ids = [1] * len(servos)
            log.info("Canales actualizados para SIMPLE PCA9685")
        else:
            # Dividir servos entre 2 PCAs
            channels = self._enable_channels_dual()
            mid = self._SERVOS_BY_DUAL_PCA
            if len(servos) != 12:
                raise ValueError(
                    f"Con 2 PCAs se requieren exactamente 12 servos."
                    f"Actualmente hay {len(self._servo_names)}"
                )
            pca_ids = [1] * mid + [2] * mid
            log.info("Canales actualizados para DUAL PCA9685")

        # Validar que hay suficientes canales para todos los servos
        if len(servos) != len(channels):
            raise ValueError(
                f"Servos ({len(servos)}) excede canales diponibles ({len(channels)})"
            )
        for name, pca, channel in zip(servos, pca_ids, channels):
            self._repository.update_servo(name, pca9685=pca, channel=channel)


if __name__ == "__main__":
    from src.utils.servo_repository import ServoRepository
    from src.utils.toml_loader import TomlLoader

    repo = ServoRepository()
    loader = TomlLoader(repo)
    ch_cfg = ChannelsConfigurator(repo)
    ch_cfg.update_channels()
    loader.synchronize()
