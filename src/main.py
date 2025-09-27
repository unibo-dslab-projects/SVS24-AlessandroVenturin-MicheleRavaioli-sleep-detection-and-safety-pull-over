import os
import time
from typing import cast

import numpy as np
import pygame
from carla import Client, Image, Location, Rotation, Sensor, Transform, Vehicle

from remove_vehicles_and_sensors import remove_vehicles_and_sensors
from vehicle_state_machine import VehicleStateMachine

FRAMERATE = 60
DT = 1 / FRAMERATE

class IO:
    _surface: pygame.Surface
    _game_display: pygame.Surface

    def __init__(self, width: int, height: int):
        init_image = np.zeros((height, width, 3), dtype="uint8")
        self._surface = pygame.surfarray.make_surface(init_image.swapaxes(0, 1))  # pyright: ignore[reportUnknownMemberType]

        # Initialise the display
        _ = pygame.init()
        self._game_display = pygame.display.set_mode(
            (image_w, image_h), pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        _ = self._game_display.blit(self._surface, (0, 0))
        pygame.display.flip()

    def prepare_output_image(self, data: Image):
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


host = os.environ.get("HOST", "localhost")
port = os.environ.get("PORT", "2000")
port = int(port)
client = Client(host, port)
client.set_timeout(120)  # pyright: ignore[reportUnknownMemberType]
world = client.get_world()
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = DT
_ = world.apply_settings(settings)

spectator = world.get_spectator()
blueprint = world.get_blueprint_library()
vehicle_bp = blueprint.filter("vehicle.*")[0]
spawn_point = world.get_map().get_spawn_points()[0]

# Initialise the camera floating behind the vehicle
camera_init_trans = Transform(Location(x=-5, z=3), Rotation(pitch=-20))
camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")

# Get camera dimensions
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()

# Instantiate objects for rendering and vehicle control
io = IO(image_w, image_h)

# Start camera with PyGame callback

try:
    vehicle = cast(Vehicle, world.spawn_actor(vehicle_bp, spawn_point))
    spectator.set_transform(  # pyright: ignore[reportUnknownMemberType]
        Transform(
            Location(
                x=spawn_point.location.x - 5,
                y=spawn_point.location.y,
                z=spawn_point.location.z + 3,
            ),
            Rotation(pitch=-20),
        )
    )
    # Spawn camera and attach it to the vehicle
    camera = cast(
        Sensor, world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)
    )
    camera.listen(lambda image: io.prepare_output_image(cast(Image, image)))
    state_machine = VehicleStateMachine(vehicle, enable_logging=True)
    should_exit = False
    while not should_exit:
        _ = world.tick()
        tick_start = time.time()

        events = io.update()
        for event in events:
            # If the window is closed, break the while loop
            if event.type == pygame.QUIT:
                should_exit = True

        state_machine.step(DT)
        compute_time = time.time() - tick_start
        if compute_time < DT:
            time.sleep(DT - compute_time)
finally:
    io.quit()
    remove_vehicles_and_sensors(world)
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = 0
    _ = world.apply_settings(settings)
