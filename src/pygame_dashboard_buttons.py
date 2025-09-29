from collections.abc import Sequence
from typing import cast, final

import pygame


@final
class PygameDashboardButtons:
    @property
    def lane_keeping_button_pressed(self):
        return self._lane_keeping_button_pressed

    @property
    def force_pullover_button_pressed(self):
        return self._force_pullover_button_pressed

    def __init__(self):
        self._lane_keeping_button_pressed = False
        self._force_pullover_button_pressed = False

    def update(self, events: Sequence[pygame.event.Event]):
        # Reset state
        self._lane_keeping_button_pressed = False
        self._force_pullover_button_pressed = False

        # Process events
        for event in events:
            if event.type == pygame.KEYDOWN:
                key = cast(int, event.key)
                if key == pygame.K_l:
                    self._lane_keeping_button_pressed = True
                if key == pygame.K_i:
                    self._force_pullover_button_pressed = True
