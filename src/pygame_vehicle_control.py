from collections.abc import Sequence
from enum import StrEnum, auto
from typing import cast, final

import pygame
from carla import Vehicle, VehicleControl


class Direction(StrEnum):
    FORWARD = auto()
    BACKWARD = auto()


@final
class PygameVehicleControl:
    @property
    def vehicle_control(self):
        return self._control

    def __init__(self, vehicle: Vehicle):
        # Conrol parameters to store the control state
        self._vehicle = vehicle
        self._steer = 0
        self._forward = False
        self._backward = False
        self._steer = None
        self._steer_cache = 0
        self._control = VehicleControl()

    def update(self, dt: float, events: Sequence[pygame.event.Event]):
        # Consume events
        for event in events:
            if event.type == pygame.KEYDOWN:
                key = cast(int, event.key)
                if key == pygame.K_UP:
                    self._forward = True
                if key == pygame.K_DOWN:
                    self._backward = True
                if key == pygame.K_RIGHT:
                    self._steer = 1
                if key == pygame.K_LEFT:
                    self._steer = -1
            if event.type == pygame.KEYUP:
                key = cast(int, event.key)
                if key == pygame.K_UP:
                    self._forward = False
                if key == pygame.K_DOWN:
                    self._backward = False
                if key == pygame.K_RIGHT:
                    self._steer = None
                if key == pygame.K_LEFT:
                    self._steer = None

        velocity = self._vehicle.get_velocity()
        direction = None
        if velocity.length() > 0.1:
            # dot > 0 if vehicle is going forward
            # dot < 0 otherwise
            dot = velocity.dot(self._vehicle.get_transform().get_forward_vector())
            if dot > 0:
                direction = Direction.FORWARD
            else:
                direction = Direction.BACKWARD

        # Reset controls
        self._control.brake = 0
        self._control.throttle = 0

        match direction:
            case None:
                if self._forward:
                    self._control.reverse = False
                if self._backward:
                    self._control.reverse = True
                self._control.throttle = 1 if self._forward or self._backward else 0
            case Direction.FORWARD:
                if self._forward:
                    self._control.throttle = 1
                if self._backward:
                    self._control.brake = 1
            case Direction.BACKWARD:
                if self._backward:
                    self._control.throttle = 1
                if self._forward:
                    self._control.brake = 1

        if self._steer is not None:
            if self._steer == 1:
                self._steer_cache += 0.9 * dt
            if self._steer == -1:
                self._steer_cache -= 0.9 * dt
            # In the example the following expression was not assigned to anything
            # min(0.7, max(-0.7, self._steer_cache))
            self._control.steer = round(self._steer_cache, 1)
        else:
            if self._steer_cache > 0.0:
                self._steer_cache *= 0.2 * dt
            if self._steer_cache < 0.0:
                self._steer_cache *= 0.2 * dt
            if 0.01 > self._steer_cache > -0.01:
                self._steer_cache = 0.0
            self._control.steer = round(self._steer_cache, 1)
