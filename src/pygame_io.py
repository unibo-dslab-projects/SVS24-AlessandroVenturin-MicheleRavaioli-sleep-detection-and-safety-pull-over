import carla
import numpy as np
import pygame


class PygameIO:
    _surface: pygame.Surface
    _game_display: pygame.Surface

    def __init__(self, width: int, height: int):
        init_image = np.zeros((height, width, 3), dtype="uint8")
        self._surface = pygame.surfarray.make_surface(init_image.swapaxes(0, 1))  # pyright: ignore[reportUnknownMemberType]

        # Initialise the display
        _ = pygame.init()
        self._game_display = pygame.display.set_mode(
            (width, height), pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        _ = self._game_display.blit(self._surface, (0, 0))
        pygame.display.flip()

    def prepare_output_image(self, data: carla.Image):
        """
        Sets the image to be rendered on the next call to update
        """
        # Camera sensor callback, reshapes raw data from camera into 2D RGB and applies to PyGame surface
        img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
        img = img[:, :, :3]
        img = img[:, :, ::-1]
        self._surface = pygame.surfarray.make_surface(img.swapaxes(0, 1))  # pyright: ignore[reportUnknownMemberType]

    def update(self) -> list[pygame.event.Event]:
        """
        Renders the prepared image and collects the queued events
        """
        _ = self._game_display.blit(self._surface, (0, 0))
        pygame.display.flip()
        # Collect key press events
        return pygame.event.get()

    def quit(self):
        pygame.quit()
