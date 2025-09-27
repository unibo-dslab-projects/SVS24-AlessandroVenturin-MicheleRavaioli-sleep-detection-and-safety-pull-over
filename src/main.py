import os
import time
from typing import cast, final
from carla import Client, Sensor, Vehicle, Transform, Location, Rotation
import numpy as np
import pygame

from remove_vehicles_and_sensors import remove_vehicles_and_sensors
from vehicle_state_machine import VehicleStateMachine

FRAMERATE = 60
DT = 1 / FRAMERATE

# Render object to keep and pass the PyGame surface
@final
class RenderObject:
    def __init__(self, width: int, height: int):
        init_image = np.zeros((height, width, 3), dtype='uint8')
        self.surface = pygame.surfarray.make_surface(init_image.swapaxes(0,1))

# Camera sensor callback, reshapes raw data from camera into 2D RGB and applies to PyGame surface
def pygame_callback(data, obj):
    img = np.reshape(np.copy(data.raw_data), (data.height, data.width, 4))
    img = img[:,:,:3]
    img = img[:, :, ::-1]
    obj.surface = pygame.surfarray.make_surface(img.swapaxes(0,1))


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

try:
    spectator = world.get_spectator()
    blueprint = world.get_blueprint_library()
    vehicle_bp = blueprint.filter("vehicle.*")[0]
    spwan_point = world.get_map().get_spawn_points()[0]
    vehicle = cast(Vehicle, world.spawn_actor(vehicle_bp, spwan_point))
    spectator.set_transform(vehicle.get_transform())  # pyright: ignore[reportUnknownMemberType]

    # Initialise the camera floating behind the vehicle
    camera_init_trans = Transform(Location(x=-5, z=3), Rotation(pitch=-20))
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')

    # Spawn camera and attach it to the vehicle
    camera = cast(
        Sensor, world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)
    )

    # Start camera with PyGame callback
    camera.listen(lambda image: pygame_callback(image, renderObject))

    # Get camera dimensions
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()

    # Instantiate objects for rendering and vehicle control
    renderObject = RenderObject(image_w, image_h)

    # Initialise the display
    _ = pygame.init()
    gameDisplay = pygame.display.set_mode((image_w,image_h), pygame.HWSURFACE | pygame.DOUBLEBUF)
    # Draw black to the display
    _ = gameDisplay.fill((0,0,0))
    _ = gameDisplay.blit(renderObject.surface, (0,0))
    pygame.display.flip()

    state_machine = VehicleStateMachine(vehicle)
    should_exit = False
    while not should_exit:
        _ = world.tick()
        tick_start = time.time()

        # Update the display
        _ = gameDisplay.blit(renderObject.surface, (0,0))
        pygame.display.flip()
        # Collect key press events
        for event in pygame.event.get():
            # If the window is closed, break the while loop
            if event.type == pygame.QUIT:
                should_exit = True

        state_machine.step(DT)
        compute_time = time.time() - tick_start
        if compute_time < DT:
            time.sleep(DT - compute_time)
finally:
    pygame.quit()
    remove_vehicles_and_sensors(world)
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = 0
    _ = world.apply_settings(settings)
