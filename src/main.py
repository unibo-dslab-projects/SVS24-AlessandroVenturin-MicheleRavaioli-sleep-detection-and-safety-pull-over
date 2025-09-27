import os
import time
from typing import cast

import pygame
from carla import Client, Image, Location, Rotation, Sensor, Transform, Vehicle

from remove_vehicles_and_sensors import remove_vehicles_and_sensors
from vehicle_state_machine import VehicleStateMachine
from pygame_io import PygameIO

FRAMERATE = 60
DT = 1 / FRAMERATE

host = os.environ.get("HOST", "localhost")
port = os.environ.get("PORT", "2000")
port = int(port)
client = Client(host, port)
client.set_timeout(120)  # pyright: ignore[reportUnknownMemberType]

# Set simulator to synchronous mode and fixed time step
world = client.get_world()
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = DT
_ = world.apply_settings(settings)

blueprint_lib = world.get_blueprint_library()
vehicle_bp = blueprint_lib.filter("vehicle.*")[0]
vehicle_spawn_point = world.get_map().get_spawn_points()[0]
camera_bp = blueprint_lib.find("sensor.camera.rgb")
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()

io = PygameIO(image_w, image_h)

try:
    # Spawn vehicle and move spectator behind it
    vehicle = cast(Vehicle, world.spawn_actor(vehicle_bp, vehicle_spawn_point))
    spectator = world.get_spectator()
    spectator.set_transform(  # pyright: ignore[reportUnknownMemberType]
        Transform(
            Location(
                x=vehicle_spawn_point.location.x - 5,
                y=vehicle_spawn_point.location.y,
                z=vehicle_spawn_point.location.z + 3,
            ),
            Rotation(pitch=-20),
        )
    )

    # Spawn camera and attach it to the vehicle
    camera = cast(
        Sensor,
        world.spawn_actor(
            camera_bp,
            Transform(Location(x=-5, z=3), Rotation(pitch=-20)),
            attach_to=vehicle,
        ),
    )

    # Bind camera to pygame window
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

    # Set simulator back to asynchronous mode and variable time step
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = 0
    _ = world.apply_settings(settings)
