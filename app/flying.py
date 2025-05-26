from time import sleep
from .driver import CopterDriver
from .schemas import FlyingMode, Coordinates, settings


def flying():
    """
    Function to simulate flying.
    """
    with CopterDriver(ip=settings.UAV_CONNECTION_STRING) as driver:
        driver.wait_for_ready()
        driver.change_mode(FlyingMode.ALT_HOLD)
        driver.arm()
        driver.takeoff(100)

        driver.goto_waypoint(
            target=Coordinates(latitude=50.443326, longitude=30.448078),
            min_distance=20.0,
        )
        driver.rotate(angle=350)
        sleep(30)
        driver.change_mode(FlyingMode.LAND)
