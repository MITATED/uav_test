from enum import Enum
from pydantic import BaseModel, Field
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """
    Application settings class
    """

    UAV_CONNECTION_STRING: str


class FlyingMode(str, Enum):
    STABILIZE = "STABILIZE"
    ACRO = "ACRO"
    ALT_HOLD = "ALT_HOLD"
    AUTO = "AUTO"
    GUIDED = "GUIDED"
    LOITER = "LOITER"
    RTL = "RTL"
    LAND = "LAND"


class Coordinates(BaseModel):
    latitude: float = Field(..., description="Latitude in degrees")
    longitude: float = Field(..., description="Longitude in degrees")


class Position(BaseModel):
    coord: Coordinates = Field(..., description="GPS coordinates of the position")
    altitude: float = Field(..., description="Altitude in meters")
    heading: float = Field(..., description="Heading in degrees")


settings = Settings()
