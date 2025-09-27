from collections.abc import Sequence
from typing import cast, final

import pygame
from carla import Vehicle, VehicleControl


@final
class PygameVehicleControl:
    @property
    def vehicle_control(self):
        return self._control

    def __init__(self, vehicle: Vehicle):
        # Conrol parameters to store the control state
        self._vehicle = vehicle
        self._steer = 0
        self._throttle = False
        self._brake = False
        self._reverse = False
        self._steer = None
        self._steer_cache = 0
        self._control = VehicleControl()

    def update(self, events: Sequence[pygame.event.Event]):
        # Consume events
        for event in events:
            if event.type == pygame.KEYDOWN:
                key = cast(int, event.key)
                if key == pygame.K_UP:
                    self._throttle = True
                if key == pygame.K_DOWN:
                    self._brake = True
                if key == pygame.K_RIGHT:
                    self._steer = 1
                if key == pygame.K_LEFT:
                    self._steer = -1
            if event.type == pygame.KEYUP:
                key = cast(int, event.key)
                if key == pygame.K_UP:
                    self._throttle = False
                if key == pygame.K_DOWN:
                    self._brake = False
                    self._reverse = False
                if key == pygame.K_RIGHT:
                    self._steer = None
                if key == pygame.K_LEFT:
                    self._steer = None

        # Compute vehicle control
        if self._throttle:
            self._control.throttle = min(self._control.throttle + 0.01, 1)
            self._control.gear = 1
            self._control.brake = False
        elif not self._brake:
            self._control.throttle = 0.0

        if self._brake:
            # If the down arrow is held down when the car is stationary, switch to reverse
            velocity = self._vehicle.get_velocity().length()
            reverse = self._vehicle.get_control().reverse
            if velocity < 0.01 and not reverse:
                self._control.brake = 0.0
                self._control.gear = 1
                self._control.reverse = True
                self._control.throttle = min(self._control.throttle + 0.1, 1)
            elif self._control.reverse:
                self._control.throttle = min(self._control.throttle + 0.1, 1)
            else:
                self._control.throttle = 0.0
                self._control.brake = min(self._control.brake + 0.3, 1)
        else:
            self._control.brake = 0.0

        if self._steer is not None:
            if self._steer == 1:
                self._steer_cache += 0.03
            if self._steer == -1:
                self._steer_cache -= 0.03
            # In the example the following expression was not assigned to anything
            # min(0.7, max(-0.7, self._steer_cache))
            self._control.steer = round(self._steer_cache, 1)
        else:
            if self._steer_cache > 0.0:
                self._steer_cache *= 0.2
            if self._steer_cache < 0.0:
                self._steer_cache *= 0.2
            if 0.01 > self._steer_cache > -0.01:
                self._steer_cache = 0.0
            self._control.steer = round(self._steer_cache, 1)
