import os
import time
from typing import cast

from carla import AttachmentType, Client, Image, Location, Rotation, Sensor, Transform, Vehicle

from inattention.detector import WebcamCameraStream
from pygame_io import PygameIO
from remove_vehicles_and_sensors import remove_vehicles_and_sensors
from vehicle_logging_config import VehicleLoggingConfig
from vehicle_state_machine import VehicleParams, VehicleStateMachine

FRAMERATE = 20
DT = 1 / FRAMERATE
MAP = "Town12"
HIGHWAY_SPAWN_POINT = Transform(Location(x=1473, y=3077.5, z=365), Rotation(yaw=180))
USE_PYGAME_CAMERA = False
CAMERA_LOCATION_OFFSET = Location(x=-5, z=3)
CAMERA_PITCH = -20
SENSORS_MAX_RANGE = 50
RADAR_SCAN_WIDTH = 3

spawn_point = HIGHWAY_SPAWN_POINT
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

spectator = world.get_spectator()
spectator.set_location(HIGHWAY_SPAWN_POINT.location)  # pyright: ignore[reportUnknownMemberType]

_ = world.tick()

# Set simulator to synchronous mode and fixed time step
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = DT
_ = world.apply_settings(settings)
traffic_manager = client.get_trafficmanager()
traffic_manager.set_synchronous_mode(True)  # pyright: ignore[reportUnknownMemberType]

blueprint_lib = world.get_blueprint_library()
vehicle_bp = blueprint_lib.filter("vehicle.*")[0]
camera_bp = blueprint_lib.find("sensor.camera.rgb")
radar_bp = world.get_blueprint_library().find('sensor.other.radar')

# Right-size radar calibration
radar_bp.set_attribute('horizontal_fov', str(85))
radar_bp.set_attribute('vertical_fov', str(2))
radar_bp.set_attribute('range', str(SENSORS_MAX_RANGE))
radar_bp.set_attribute('points_per_second', str(1000))
radar_location = Location(x=2.0, z=1.0)
radar_rotation = Rotation(yaw=50)
radar_transform = Transform(radar_location,radar_rotation)

pygame_window_width = camera_bp.get_attribute("image_size_x").as_int()
pygame_window_height = camera_bp.get_attribute("image_size_y").as_int()
io = PygameIO(pygame_window_width, pygame_window_height)

try:
    # Spawn vehicle
    vehicle = cast(Vehicle, world.spawn_actor(vehicle_bp, spawn_point))

    # Spawn obstacles
    _ = cast(
        Vehicle,
        world.spawn_actor(
            vehicle_bp,
            Transform(
                Location(spawn_point.location + Location(x=-150, y=-2.5)),
                spawn_point.rotation,
            ),
        ),
    )

    if USE_PYGAME_CAMERA:
        camera = cast(Sensor, world.spawn_actor(camera_bp, Transform()))

        # Bind camera to pygame window
        camera.listen(lambda image: io.prepare_output_image(cast(Image, image)))

    # Getting driver camera (webcam)
    driver_camera_stream = WebcamCameraStream(device=0, width=600, height=480)

    # Spawn radar
    front_radar = cast(
        Sensor,
        world.spawn_actor(
            radar_bp,
            radar_transform,
            attach_to=vehicle,
            attachment_type=AttachmentType.Rigid,
        ),
    )

    state_machine = VehicleStateMachine(
        pygame_io=io,
        vehicle=vehicle,
        world=world,
        map=map,
        traffic_manager=traffic_manager,
        params=VehicleParams(
            sensors_max_range=SENSORS_MAX_RANGE,
            cruise_target_speed_kmh=100,
            max_pull_over_acceleration=-2.0,
            radar_scan_width=RADAR_SCAN_WIDTH,
        ),
        driver_camera_stream=driver_camera_stream,
        front_radar=front_radar,
        logging_config=VehicleLoggingConfig(log_entries=True),
    )

    def move_to_with_local_offsets(target:Transform, location_offset: Location,rotation_offset: Rotation):
        target = vehicle.get_transform()
        computed_rotation = target.rotation
        computed_rotation.pitch += rotation_offset.pitch
        computed_rotation.roll += rotation_offset.roll
        computed_rotation.yaw += rotation_offset.yaw

        # Get local axis vectors
        forward_vector = target.get_forward_vector()
        right_vector = target.get_right_vector()
        up_vector = target.get_up_vector()

        # Apply the offset in the vehicle's local coordinate space
        global_location_offset = (
            forward_vector * location_offset.x
            + right_vector * location_offset.y
            + up_vector * location_offset.z
        )
        computed_location = target.location
        computed_location = Location(computed_location + global_location_offset)
        return Transform(computed_location, computed_rotation)

    should_exit = False
    while not should_exit:
        tick_start = time.time()
        _ = world.tick()

        # Moving camera, radar and spectator to the vehicle
        # Attaching camera and radar to the vehicle caused carla to crash
        vehicle_transform = vehicle.get_transform()
        camera_transform = move_to_with_local_offsets(
            vehicle_transform,
            CAMERA_LOCATION_OFFSET,
            Rotation(pitch=CAMERA_PITCH),
        )
        if camera is not None:
            camera.set_transform(camera_transform)  # pyright: ignore[reportUnknownMemberType]
        spectator.set_transform(camera_transform)  # pyright: ignore[reportUnknownMemberType]
        world.debug.draw_string(front_radar.get_transform().location, "R")

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
    traffic_manager.set_synchronous_mode(False)  # pyright: ignore[reportUnknownMemberType]
