from dronekit import Vehicle
from typing import Optional
from .schemas import FlyingMode, Position, Coordinates


class Telemetry:
    vehicle: Vehicle

    def get_position(self) -> Position:
        """Get the current position of the vehicle."""
        location = self.vehicle.location.global_relative_frame
        return Position(
            coord=Coordinates(
                latitude=location.lat or 0,
                longitude=location.lon or 0,
            ),
            altitude=location.alt or 0,
            heading=self.vehicle.heading or 0,
        )

    def get_mode(self) -> Optional[FlyingMode]:
        """Get the current mode of the vehicle."""
        return FlyingMode(self.vehicle.mode.name) if self.vehicle.mode else None

    def is_armed(self) -> bool:
        """Check if the vehicle is armed."""
        return self.vehicle.armed

    def is_armable(self) -> bool:
        """Check if the vehicle can be armed."""
        return self.vehicle.is_armable
