import os
import time
from typing import cast

from carla import Client, Image, Location, Rotation, Sensor, Transform, Vehicle

from inattention.detector import WebcamCameraStream
from pygame_io import PygameIO
from remove_vehicles_and_sensors import remove_vehicles_and_sensors
from vehicle_logging_config import VehicleLoggingConfig
from vehicle_state_machine import VehicleStateMachine

FRAMERATE = 20
DT = 1 / FRAMERATE
MAP = "Town12"
HIGHWAY_SPAWN_POINT = Transform(Location(x=1473, y=3077.5, z=365), Rotation(yaw=180))
HIGHWAY_DESTINATION = Location(x=448, y=3079, z=361)
USE_PYGAME_CAMERA = False
CAMERA_LOCATION_OFFSET = Location(x=-5, z=3)
CAMERA_PITCH = -20

spawn_point = HIGHWAY_SPAWN_POINT
destination = HIGHWAY_DESTINATION
camera: Sensor | None = None

host = os.environ.get("HOST", "localhost")
port = os.environ.get("PORT", "2000")
port = int(port)
client = Client(host, port)
client.set_timeout(120)  # pyright: ignore[reportUnknownMemberType]

world = client.get_world()
map = world.get_map()
if not map.name.endswith(MAP):
    world = client.load_world(MAP)
    map = world.get_map()

# Set simulator to synchronous mode and fixed time step
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = DT
_ = world.apply_settings(settings)

blueprint_lib = world.get_blueprint_library()
vehicle_bp = blueprint_lib.filter("vehicle.*")[0]
camera_bp = blueprint_lib.find("sensor.camera.rgb")
pygame_window_width = camera_bp.get_attribute("image_size_x").as_int()
pygame_window_height = camera_bp.get_attribute("image_size_y").as_int()

io = PygameIO(pygame_window_width, pygame_window_height)

try:
    # Spawn vehicle and move spectator behind it
    vehicle = cast(Vehicle, world.spawn_actor(vehicle_bp, spawn_point))

    if USE_PYGAME_CAMERA:
        camera = cast(Sensor, world.spawn_actor(camera_bp, Transform()))

        # Bind camera to pygame window
        camera.listen(lambda image: io.prepare_output_image(cast(Image, image)))


    # Getting driver camera (webcam)
    driver_camera_stream = WebcamCameraStream(device=0, width=600, height=480)

    _ = world.tick()  # fixes bug: https://github.com/carla-simulator/carla/issues/2634

    spectator = world.get_spectator()
    spectator.set_location(vehicle.get_location())  # pyright: ignore[reportUnknownMemberType]

    state_machine = VehicleStateMachine(
        pygame_io=io,
        vehicle=vehicle,
        destination=destination,
        map=map,
        meters_for_safe_pullover=50,
        driver_camera_stream=driver_camera_stream,
        logging_config=VehicleLoggingConfig(log_entries=True),
    )

    should_exit = False
    while not should_exit:
        tick_start = time.time()
        _ = world.tick()

        # Moving camera and spectator to the vehicle
        # TODO: factor out this code into a function
        vehicle_transform = vehicle.get_transform()
        camera_rotation = vehicle_transform.rotation
        camera_rotation.pitch += CAMERA_PITCH

        # Get local axis vectors
        forward_vector = vehicle_transform.get_forward_vector()
        right_vector = vehicle_transform.get_right_vector()
        up_vector = vehicle_transform.get_up_vector()

        # Apply the offset in the vehicle's local coordinate space
        offset = (
            forward_vector * CAMERA_LOCATION_OFFSET.x +
            right_vector   * CAMERA_LOCATION_OFFSET.y +
            up_vector      * CAMERA_LOCATION_OFFSET.z
        )
        camera_location = vehicle_transform.location
        camera_location = Location(camera_location + offset)
        camera_transform = Transform(camera_location, camera_rotation)
        if camera is not None:
            # Attaching the camera to vehicle caused carla to crash
            camera.set_transform(camera_transform)  # pyright: ignore[reportUnknownMemberType]
        spectator.set_transform(camera_transform)  # pyright: ignore[reportUnknownMemberType]

        world.debug.draw_string(destination, "Destination")
        compute_time = time.time() - tick_start
        should_exit = not state_machine.step(DT)
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
