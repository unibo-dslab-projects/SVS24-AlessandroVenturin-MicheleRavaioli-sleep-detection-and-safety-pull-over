import os
import random
import time
import math
from typing import cast

import pygame
import carla
from carla import Client, Color, Image, Location, Rotation, Sensor, Transform, Vehicle, World

from remove_vehicles_and_sensors import remove_vehicles_and_sensors
from vehicle_state_machine import VehicleStateMachine
from pygame_io import PygameIO

from pullover.checker import SafePulloverChecker

FRAMERATE = 30
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

def attach_radar(vehicle: Vehicle):
    global world
    # --------------
    # Add a new radar sensor to my ego
    # --------------
    rad_bp = world.get_blueprint_library().find('sensor.other.radar')

    # Right-size calibration
    rad_bp.set_attribute('horizontal_fov', str(85))
    rad_bp.set_attribute('vertical_fov', str(2))
    rad_bp.set_attribute('range', str(100))
    rad_bp.set_attribute('points_per_second', str(1000))
    rad_location = carla.Location(x=2.0, z=1.0)
    rad_rotation = carla.Rotation(yaw=50)
    rad_transform = carla.Transform(rad_location,rad_rotation)


    rad_ego = cast(Sensor, world.spawn_actor(rad_bp,rad_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid))

    def _rad_callback(radar_data: carla.RadarMeasurement):
        velocity_range = 7.5 # m/s
        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            world.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))
    
    # rad_ego.listen(lambda data: _rad_callback(cast(carla.RadarMeasurement, data)))
    print("attached radar!")
    return rad_ego

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

    radar = attach_radar(vehicle)

    offset = float(vehicle.bounding_box.extent.y)
    pcheck = SafePulloverChecker(radar, vehicle, 3.0, offset, safety_delay=1.0, debug=True)

    # Bind camera to pygame window
    camera.listen(lambda image: io.prepare_output_image(cast(Image, image)))
    destination_trans = random.choice(world.get_map().get_spawn_points())

    destination = destination_trans.location
    state_machine = VehicleStateMachine(
        pygame_io=io,
        vehicle=vehicle,
        destination=destination,
        enable_logging=False,
    )

    print('starting...')
    should_exit = False
    while not should_exit:
        tick_start = time.time()
        _ = world.tick()
        should_exit = not state_machine.step(DT)
        print('pullover:', 'safe' if pcheck.is_pullover_safe() else 'NOOO!!')
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
