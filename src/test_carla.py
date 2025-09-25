import time
import carla
from typing import cast, override
from carla import Vehicle, Vector3D, VehicleControl
from remove_vehicles_and_sensors import remove_vehicles_and_sensors
from state_machine.sync_state_machine import State, StateAction, Context, SyncStateMachine, Transition

framerate = 60
dt = 1 / framerate


type CarTimers = None
type CarContext = Context[CarTimers]


class CarData:
    speed: Vector3D = Vector3D()
    car_actor: Vehicle
    vehicle_control: VehicleControl

    def __init__(
        self, car_actor: Vehicle, vehicle_control: VehicleControl
    ) -> CarTimers:
        self.car_actor = car_actor
        self.vehicle_control = vehicle_control


class CarState(State[CarData, CarTimers]): ...


class CarStateAction(StateAction[CarData, CarTimers]): ...


class CarTransition(Transition[CarData, CarTimers]): ...


class Car(CarState):
    @override
    def children(self) -> list[CarState]:
        return [Accelerating(), Decelerating()]

    @override
    def on_do(self, data: CarData, ctx: CarContext):
        data.speed = data.car_actor.get_velocity()
        data.vehicle_control = VehicleControl()


class Accelerating(CarState):
    @override
    def on_do(self, data: CarData, ctx: CarContext):
        data.vehicle_control.throttle = 1
        data.car_actor.apply_control(data.vehicle_control)

    @override
    def transitions(self) -> list[CarTransition]:
        return [
            CarTransition(
                to=Decelerating(), condition=lambda data, ctx: data.speed.length() > 18
            )
        ]


class Decelerating(CarState):
    @override
    def on_do(self, data: CarData, ctx: CarContext):
        data.vehicle_control.brake = 1
        data.car_actor.apply_control(data.vehicle_control)

    @override
    def transitions(self) -> list[CarTransition]:
        return [
            CarTransition(
                to=Accelerating(), condition=lambda data, ctx: data.speed.length() <= 0
            )
        ]


class CarStateMachine(SyncStateMachine[CarData, CarTimers]):
    def __init__(self, data: CarData):
        super().__init__([Car()], data)


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(120)
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = dt
    _ = world.apply_settings(settings)
    spectator = world.get_spectator()
    blueprint = world.get_blueprint_library()
    car_bp = blueprint.filter("vehicle.*")[0]
    spwan_point = world.get_map().get_spawn_points()[0]
    car = cast(Vehicle, world.spawn_actor(car_bp, spwan_point))
    vehicle_control = carla.VehicleControl()
    car_data = CarData(car, vehicle_control)
    state_machine = CarStateMachine(car_data)
    spectator.set_transform(car.get_transform())
    try:
        while True:
            _ = world.tick()
            tick_start = time.time()
            state_machine.step(dt)
            compute_time = time.time() - tick_start
            if compute_time < dt:
                time.sleep(dt - compute_time)
    finally:
        remove_vehicles_and_sensors(world)
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = 0
        _ = world.apply_settings(settings)


if __name__ == "__main__":
    main()
