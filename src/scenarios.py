
import random
import math
from types import FunctionType
import carla
from typing import List, Tuple, cast
from carla import Client, World, Map, Vehicle, Transform, Location, Rotation, VehicleControl
from remove_vehicles_and_sensors import remove_vehicles_and_sensors

class Scenario():
    def __init__(self, 
                 map_name: str,
                 spawn_point: Transform,
                 cruise_control_speed: float = 100,
                 stopped_cars: list[tuple[str, Transform]] = [], # [(blueprint_id, spawn_point)...]
                 moving_cars: list[tuple[str, Transform, float | None]] = []): # [(blueprint_id, spawn_point, speed)...]
        self.map_name     = map_name
        self.spawn_point  = spawn_point
        self.stopped_cars = stopped_cars
        self.moving_cars  =  moving_cars
        self.cruise_control_speed = cruise_control_speed


    def load(self, client: Client, ego_vehicle: Vehicle):
        """
        Load this scenario into the world, and setup the ego vehicle accordingly. 
        Throws error if the map isn't the correct one.
        """
        # Load map
        world = client.get_world()
        map = world.get_map()
        if not map.name.endswith(self.map_name):
            raise Exception(f"Wrong map loaded! required map: {self.map_name}, actual map: {map.name}")
        
        spectator = world.get_spectator()
        spectator.set_location(self.spawn_point.location)

        # Create ego vehicle
        ego_vehicle.set_transform(self.spawn_point)

        blueprints = world.get_blueprint_library()
        for v, t in self.stopped_cars:
            vehicle = cast(Vehicle | None, world.try_spawn_actor(blueprints.find(v), t))
            if vehicle is None:
                continue
            c = VehicleControl()
            c.brake = 1
            vehicle.apply_control(c)

        traffic_manager = client.get_trafficmanager()
        for v, t, s in self.moving_cars:
            vehicle = cast(Vehicle | None, world.try_spawn_actor(blueprints.find(v), t))
            if vehicle is None:
                continue
            vehicle.set_autopilot(True)
            if s is not None:
                traffic_manager.set_desired_speed(vehicle, s)

    def unload(self, client: Client):
        remove_vehicles_and_sensors(client.get_world())



### SCENARIOS ###

MAP04_STRAIGHT_ROAD_SPAWN_POINT = Transform(
    Location(x=-350.343048, y=37.064049, z=0.281942),
    Rotation(pitch=0.000000, yaw=-0.368408, roll=0.000000)
)

MAP04_CURVE_ROAD_SPAWN_POINT = Transform(
    Location(x=413.543457, y=-154.245117, z=1.226235),
    Rotation(pitch=0.000000, yaw=-90.0000, roll=0.000000)
)

class EmptyStraightRoadScenario(Scenario):
    """
    Scenario with an empty straight road and some stopped cars in the emergency lane.
    """
    def __init__(self):
        super().__init__(map_name="Town04", spawn_point=MAP04_STRAIGHT_ROAD_SPAWN_POINT)
        p = self.spawn_point
        self.stopped_cars = [
            ("vehicle.chevrolet.impala", Transform(
                Location(p.location.x + 50, p.location.y + 3, p.location.z + 1),
                p.rotation
            )),
            ("vehicle.citroen.c3", Transform(
                Location(p.location.x + 50 + 7, p.location.y + 3, p.location.z + 1),
                p.rotation
            )),
            ("vehicle.audi.etron", Transform(
                Location(p.location.x + 50 + 15, p.location.y + 3, p.location.z + 1),
                p.rotation
            )),
            ("vehicle.jeep.wrangler_rubicon", Transform(
                Location(p.location.x + 50 + 20, p.location.y + 3, p.location.z + 1),
                p.rotation
            )),
            ("vehicle.volkswagen.t2_2021", Transform(
                Location(p.location.x + 50 + 40, p.location.y + 2.8, p.location.z + 2.5),
                p.rotation
            )),
            ("vehicle.chevrolet.impala", Transform(
                Location(p.location.x + 50 + 100, p.location.y + 3, p.location.z + 5),
                p.rotation
            )),
            ("vehicle.citroen.c3", Transform(
                Location(p.location.x + 50 + 107, p.location.y + 3, p.location.z + 5),
                p.rotation
            )),
            ("vehicle.audi.etron", Transform(
                Location(p.location.x + 50 + 115, p.location.y + 3, p.location.z + 6),
                p.rotation
            )),
        ]

class EmptyCurveRoadScenario(Scenario):
    """
    Scenario with an empty curved road and some stopped cars in the emergency lane.
    """
    def __init__(self):
        super().__init__(map_name="Town04", spawn_point=MAP04_CURVE_ROAD_SPAWN_POINT)

        self.stopped_cars = [
            ("vehicle.chevrolet.impala", Transform(
                Location(x=415.110840, y=-257.378632, z=1),
                Rotation(pitch=3.933704, yaw=-97.159393, roll=0.000142)
            )),
            ("vehicle.citroen.c3", Transform(
                Location(x=413.464325, y=-266.408203, z=1),
                Rotation(pitch=0.061649, yaw=-102.561653, roll=0.000136)
            )),
            ("vehicle.audi.etron", Transform(
                Location(x=407.646240, y=-287.340210, z=1),
                Rotation(pitch=0.658635, yaw=-107.898689, roll=0.000138)
            )),
            ("vehicle.jeep.wrangler_rubicon", Transform(
                Location(x=397.616119, y=-310.054382, z=1),
                Rotation(pitch=0.314066, yaw=-117.047523, roll=0.000143)
            )),
            ("vehicle.volkswagen.t2_2021", Transform(
                Location(x=358.977020, y=-358.655273, z=1),
                Rotation(pitch=0.856130, yaw=-138.922958, roll=0.000021)
            )),
            ("vehicle.chevrolet.impala", Transform(
                Location(x=350.295807, y=-365.557068, z=1),
                Rotation(pitch=0.520057, yaw=-143.702698, roll=0.000022)
            )),
        ]
        
class BusyStraightRoadScenario(EmptyStraightRoadScenario):
    """
    Scenario with a straight road with traffic and some stopped cars in the emergency lane.
    """
    def __init__(self):
        super().__init__()
        p = MAP04_STRAIGHT_ROAD_SPAWN_POINT
        self.cruise_control_speed = 25
        self.moving_cars = [
            ("vehicle.chevrolet.impala", Transform(
                Location(x=-329.602448, y=33.682590, z=1.635707),
                p.rotation
            ), None),
            ("vehicle.citroen.c3", Transform(
                Location(x=-310.013245, y=37.275883, z=1.992048),
                p.rotation
            ), 70),
            ("vehicle.audi.etron", Transform(
                Location(x=-320.013245, y=37.275883, z=1.992048),
                p.rotation
            ), 25),
            ("vehicle.jeep.wrangler_rubicon", Transform(
                Location(x=-363.131042, y=37.079632, z=1.659683),
                p.rotation
            ), 50),
        ]

class BusyCurveRoadScenario(EmptyCurveRoadScenario):
    """
    Scenario with a curved road with traffic and some stopped cars in the emergency lane.
    """
    def __init__(self):
        super().__init__()
        self.cruise_control_speed = 25
        self.moving_cars = [
            ("vehicle.chevrolet.impala", Transform(
                Location(x=410.391357, y=-179.754700, z=0.597406),
                Rotation(pitch=-0.706938, yaw=-89.009186, roll=0.000139)
            ), None),
            ("vehicle.citroen.c3", Transform(
                Location(x=413.957489, y=-192.886047, z=0.722103),
                Rotation(pitch=7.288692, yaw=-88.923775, roll=0.000118)
            ), 70),
            ("vehicle.audi.etron", Transform(
                Location(x=413.988037, y=-211.916397, z=0.450147),
                Rotation(pitch=0.129835, yaw=-87.247612, roll=0.000091)
            ), 25),
            ("vehicle.jeep.wrangler_rubicon", Transform(
                Location(x=412.690002, y=-71.731537, z=1.184738),
                Rotation(pitch=-0.058719, yaw=-86.990295, roll=0.000142)
            ), 50),
        ]

class TrafficJamScenario(EmptyStraightRoadScenario):
    """
    Scenario with a straight jammed road and some stopped cars in the emergency lane.
    """
    def __init__(self):
        super().__init__()
        p = MAP04_STRAIGHT_ROAD_SPAWN_POINT
        self.cruise_control_speed = 5
        self.moving_cars = [
            ("vehicle.chevrolet.impala", Transform(
                Location(p.location.x + 12, p.location.y, z=2),
                p.rotation
            ), 5),
            ("vehicle.citroen.c3", Transform(
                Location(p.location.x + 22, p.location.y, z=2),
                p.rotation
            ), 10),
            ("vehicle.audi.etron", Transform(
                Location(p.location.x + 32, p.location.y, z=2),
                p.rotation
            ), 5),
            ("vehicle.jeep.wrangler_rubicon", Transform(
                Location(p.location.x, p.location.y - 3, z=2),
                p.rotation
            ), 5),
            ("vehicle.volkswagen.t2_2021", Transform(
                Location(p.location.x + 12, p.location.y - 3, z=2),
                p.rotation
            ), 4.5),
            ("vehicle.chevrolet.impala", Transform(
                Location(p.location.x + 22, p.location.y - 3, z=2),
                p.rotation
            ), 4.5),
        ]