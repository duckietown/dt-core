"""Run in circle script."""

import math

from packages.duckiematrix_engine.entities.matrix_entity import (
    MatrixEntityBehavior,
)


class RunInCircleScript(MatrixEntityBehavior):
    """Run in circle script."""

    _radius: float
    _speed: float
    _time: float

    def __init__(
        self,
        matrix_key: str,
        world_key: str | None,
        radius: float = 0.2,
        speed: float = 0.5,
    ) -> None:
        """Initialize run in circle script."""
        super().__init__(matrix_key, world_key)
        self._radius = radius
        self._speed = speed
        self._time = 0

    def update(self, delta_time: float) -> None:
        """Update."""
        self._time += delta_time
        if self.state:
            angle = self._speed * self._time
            value = math.sin(angle)
            self.state.x = self.state.initial_pose["x"] + self._radius * value
            value = math.cos(angle)
            self.state.y = self.state.initial_pose["y"] + self._radius * value
            self.state.yaw = math.pi - angle
            self.state.commit()
