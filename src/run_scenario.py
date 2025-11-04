import os
import time
from argparse import ArgumentParser
from typing import cast

import pygame
from carla import (
    AttachmentType,
    Client,
    Image,
    Location,
    Rotation,
    Sensor,
    Transform,
    Vehicle,
)

import scenarios
from inattention.detector import WebcamCameraStream
from pygame_io import PygameIO
from vehicle_logging_config import VehicleLoggingConfig
from vehicle_state_machine import VehicleParams, VehicleStateMachine

parser = ArgumentParser("run_scenario")
_ = parser.add_argument(
    "scenario_index",
    help="The index of the scenario to run. defaults to 1.\n1) Empty straight road\n2) Empty curve\n3) Straight road with traffic\n4) Curve with traffic\n5) Traffic jam",
    type=int,
)


def str_or_int(value: str):
    try:
        return int(value)
    except ValueError:
        return value


_ = parser.add_argument(
    "-camera_device",
    help="Camera device to use, accept both index (ex: 0) and name (ex: /dev/video0)",
    type=str_or_int,
    default="0",
)

_ = parser.add_argument(
    "-wake_up_sound",
    help="Sound to play in order to wake up the driver.",
    type=str,
    default="media/alarm.mp3",
)

_ = parser.add_argument(
    "-fps",
    help="Set the amount of frame/steps per seconds the simulator should run. If you provide a value too high for your system you may experience a slow-motion effect. We suggest a value of 20 on low end systems",
    type=int,
    default=60,
)

_ = parser.add_argument(
    "-use_pygame_camera",
    help="Send camera feed to pygame window (heavier on system resources)",
    action="store_true",
)

args = parser.parse_args()


def choose_scenario(i: int | None) -> scenarios.Scenario:
    if i == 2:
        return scenarios.EmptyCurveRoadScenario()
    elif i == 3:
        return scenarios.BusyStraightRoadScenario()
    elif i == 4:
        return scenarios.BusyCurveRoadScenario()
    elif i == 5:
        return scenarios.TrafficJamScenario()
    return scenarios.EmptyStraightRoadScenario()


# Define scenario
scenario = choose_scenario(cast(int, args.scenario_index))

framerate = cast(int, args.fps)
DT = 1 / framerate
MAP = scenario.map_name
CAMERA_LOCATION_OFFSET = Location(x=-8, z=3)
CAMERA_PITCH = -20
SENSORS_MAX_RANGE = 50
RADAR_SCAN_WIDTH = 3

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
traffic_manager = client.get_trafficmanager()
traffic_manager.set_synchronous_mode(True)  # pyright: ignore[reportUnknownMemberType]

blueprint_lib = world.get_blueprint_library()
# Blueprints
vehicle_bp = blueprint_lib.find("vehicle.mercedes.coupe_2020")
# vehicle_bp = blueprint_lib.filter("vehicle.*")[0]
camera_bp = blueprint_lib.find("sensor.camera.rgb")
radar_bp = world.get_blueprint_library().find("sensor.other.radar")
# Right-side radar calibration
radar_bp.set_attribute("horizontal_fov", str(85))
radar_bp.set_attribute("vertical_fov", str(2))
radar_bp.set_attribute("range", str(SENSORS_MAX_RANGE))
radar_bp.set_attribute("points_per_second", str(2000))
radar_location = Location(x=2.0, z=0.2)
radar_rotation = Rotation(yaw=50)
radar_transform = Transform(radar_location, radar_rotation)

pygame_window_width = camera_bp.get_attribute("image_size_x").as_int()
pygame_window_height = camera_bp.get_attribute("image_size_y").as_int()
io = PygameIO(pygame_window_width, pygame_window_height)

spawn_point = scenario.spawn_point
spectator = world.get_spectator()
spectator.set_location(spawn_point.location)  # pyright: ignore[reportUnknownMemberType]

try:
    # Spawn vehicle
    vehicle = cast(Vehicle, world.spawn_actor(vehicle_bp, spawn_point))

    # Load scenario
    scenario.load(client, vehicle)

    if cast(bool, args.use_pygame_camera):
        camera = cast(Sensor, world.spawn_actor(camera_bp, Transform()))

        # Bind camera to pygame window
        camera.listen(lambda image: io.prepare_output_image(cast(Image, image)))

    # Getting driver camera (webcam)
    driver_camera_stream = WebcamCameraStream(
        device=cast(str | int, args.camera_device), width=600, height=480
    )

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
            cruise_target_speed_kmh=scenario.cruise_control_speed,
            max_pull_over_acceleration=-2.0,
            radar_scan_width=RADAR_SCAN_WIDTH,
        ),
        driver_camera_stream=driver_camera_stream,
        front_radar=front_radar,
        wake_up_sound=pygame.mixer.Sound(cast(str, args.wake_up_sound)),
        logging_config=VehicleLoggingConfig(log_entries=True),
    )

    def move_to_with_local_offsets(
        target: Transform, location_offset: Location, rotation_offset: Rotation
    ):
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

        compute_time = time.time() - tick_start
        should_exit = not state_machine.step(DT)
        if compute_time < DT:
            time.sleep(DT - compute_time)

finally:
    io.quit()
    # Reset
    scenario.unload(client)

    # Set simulator back to asynchronous mode and variable time step
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = 0
    _ = world.apply_settings(settings)
    traffic_manager.set_synchronous_mode(False)  # pyright: ignore[reportUnknownMemberType]
