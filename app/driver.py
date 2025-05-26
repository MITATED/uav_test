"""Copter control driver with detailed logging.

Logging is intentionally kept free of configuration to avoid polluting
applications that import this module.  The caller should configure the
`logging` package (handlers/formatters/levels) as appropriate for the
runtime environment.
"""

from __future__ import annotations

import math
import time
from typing import Optional

from dronekit import Vehicle, connect
from pymavlink import mavutil

from .logger import logger
from .schemas import Coordinates, FlyingMode
from .telemetry import Telemetry


class CopterDriver(Telemetry):
    """High‑level flight helper that controls a copter through DroneKit.

    All public operations log important steps and state changes so that the
    host application can trace behaviour (use DEBUG for verbose telemetry,
    INFO for high‑level state, WARNING/ERROR for exceptional situations).
    """

    vehicle: Vehicle
    rate_hz: float = 10.0

    # ---------------------------------------------------------------------
    # Life‑cycle helpers
    # ---------------------------------------------------------------------
    def __init__(self, ip: str):
        self.ip = ip
        logger.debug("CopterDriver initialised with endpoint %s", ip)

    def __enter__(self):
        logger.info("Connecting to vehicle @ %s…", self.ip)
        self.vehicle = connect(self.ip, baud=57600, wait_ready=True)
        self.time_boot_ms = int(time.monotonic() * 1000)
        logger.info(
            "Vehicle connected – FW: %s",
            self.vehicle.version,
        )
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        logger.info("Context exit: armed=%s, exc=%s", self.is_armed(), exc_type)
        if self.is_armed():
            logger.warning("Vehicle still armed – forcing disarm before close…")
            self.force_disarm()
            time.sleep(1)
        self.vehicle.close()
        logger.info("Vehicle connection closed")
        return False

    # ---------------------------------------------------------------------
    # DroneKit wrappers / state helpers
    # ---------------------------------------------------------------------
    def wait_for_ready(self):
        logger.info("Waiting until vehicle is armable…")
        while not self.is_armable():
            logger.debug("Vehicle not armable yet – waiting…")
            time.sleep(1)
        logger.info("Vehicle is armable ✓")

    def change_mode(self, mode: FlyingMode):
        if self.vehicle.mode != mode:
            logger.info("Changing mode from %s → %s", self.vehicle.mode, mode)
            self.vehicle.mode = mode
            while self.vehicle.mode != mode:
                logger.debug("Awaiting mode change… current=%s", self.vehicle.mode)
                time.sleep(0.1)
            logger.debug("Mode changed to %s", self.vehicle.mode)
        else:
            logger.debug("Vehicle already in mode %s – no change", mode)

    def arm(self):
        if not self.is_armed():
            logger.info("Arming vehicle…")
            self.vehicle.armed = True
            while not self.is_armed():
                logger.debug("Waiting for arm acknowledgement…")
                time.sleep(0.1)
            logger.info("Vehicle armed ✓")
        else:
            logger.debug("Vehicle already armed – skipping arm request")

    def change_radio_control(
        self,
        roll: Optional[float] = None,
        pitch: Optional[float] = None,
        throttle: Optional[float] = 1500,
        yaw: Optional[float] = None,
    ):
        """Override RC channels. Use *None* to leave a channel untouched."""

        def _pwm(val: Optional[float]) -> int:  # transition None → 0 (skip)
            return 0 if val is None else int(val)

        self.vehicle._master.mav.rc_channels_override_send(
            self.vehicle._master.target_system,
            self.vehicle._master.target_component,
            _pwm(roll),
            _pwm(pitch),
            _pwm(throttle),
            _pwm(yaw),
            0,
            0,
            0,
            0,  # CH5–CH8 (dont change)
        )
        logger.debug(
            "RC override → roll=%s pitch=%s throttle=%s yaw=%s",
            roll,
            pitch,
            throttle,
            yaw,
        )

    # ---------------------------------------------------------------------
    # Math helpers
    # ---------------------------------------------------------------------
    @staticmethod
    def calculate_distance(
        current_lattitude: float,
        current_longitude: float,
        target_lattitude: float,
        target_longitude: float,
    ) -> float:
        """Great‑circle distance (Haversine) in metres."""

        R = 6_371_000.0  # Earth radius (m)
        dlat, dlon = map(
            math.radians,
            (
                target_lattitude - current_lattitude,
                target_longitude - current_longitude,
            ),
        )
        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(math.radians(current_lattitude))
            * math.cos(math.radians(target_lattitude))
            * math.sin(dlon / 2) ** 2
        )
        distance = R * 2 * math.asin(math.sqrt(a))
        logger.debug(
            "Distance calc: (%.6f, %.6f) → (%.6f, %.6f) = %.2f m",
            current_lattitude,
            current_longitude,
            target_lattitude,
            target_longitude,
            distance,
        )
        return distance

    # ---------------------------------------------------------------------
    # High‑level flight helpers
    # ---------------------------------------------------------------------
    def takeoff(
        self,
        target_altitude: float,
        pwm_min: int = 1550,
        pwm_max: int = 3000,
        kp: float = 6.0,
        ki: float = 1.2,
        timeout_s: int = 60,
    ):
        logger.info("Take‑off – target_altitude=%.1f m", target_altitude)
        period = 1.0 / self.rate_hz
        i_term = 0.0
        start = time.time()
        prev_diff = target_altitude
        try:
            while True:
                pos = self.get_position()
                diff = target_altitude - pos.altitude  # height difference
                i_term += diff * ki * period  # integral component
                i_term = max(-1500, min(1500, i_term))  # prevent runaway

                pwm = pwm_min + kp * diff + i_term
                pwm = max(pwm_min, min(pwm_max, pwm))
                self.change_radio_control(throttle=pwm)  # Roll/Pitch/Yaw neutral

                logger.debug(
                    "Take‑off ctrl: alt=%.2f diff=%.2f pwm=%.0f",
                    pos.altitude,
                    diff,
                    pwm,
                )

                # exit condition
                if (
                    pos.altitude >= target_altitude * 0.95
                    and abs(diff - prev_diff) < 0.05
                ):
                    logger.info(
                        "Reached target altitude %.2f m (current %.2f m)",
                        target_altitude,
                        pos.altitude,
                    )
                    break

                prev_diff = diff
                time.sleep(period)

                if time.time() - start > timeout_s:
                    logger.error("Take‑off timed‑out after %s s", timeout_s)
                    raise TimeoutError("Takeoff exceeded time limit")
        finally:
            self.change_radio_control()
            logger.debug("Throttle neutralised after take‑off")

    @staticmethod
    def angle(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Bearing from point 1 → point 2 (degrees clockwise from North)."""
        y = math.sin(math.radians(lon2 - lon1)) * math.cos(math.radians(lat2))
        x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(
            math.radians(lat1)
        ) * math.cos(math.radians(lat2)) * math.cos(math.radians(lon2 - lon1))
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
        logger.debug(
            "Bearing calc: (%.6f, %.6f) → (%.6f, %.6f) = %.1f°",
            lat1,
            lon1,
            lat2,
            lon2,
            bearing,
        )
        return bearing

    def rotate(
        self,
        angle: float,
        error_tolerance: float = 5.0,
        timeout_s: int = 60,
    ):
        logger.info(
            "Rotating %s by %.1f° (tolerance=%.1f°)",
            "clockwise" if angle > 0 else "counter‑clockwise",
            abs(angle),
            error_tolerance,
        )
        minimum_sensitivity = 20  # minimal yaw sensitivity
        start = time.time()
        first_success_check = None
        while True:
            pos = self.get_position()
            current_heading = pos.heading
            diff = (angle - current_heading + 540) % 360 - 180
            yaw_direction = -1 if diff < 0 else 1
            diff = abs(diff)
            if diff < 5:
                if first_success_check is None:
                    # first successful check, start timer
                    logger.debug("First successful check, starting timer")
                    first_success_check = time.time()
                    time.sleep(1.0 / self.rate_hz)
                    continue  # wait for next check
                if time.time() - first_success_check > 1.0:
                    logger.debug(
                        "Yaw diff=%.1f° (heading=%.1f°) – checking again",
                        diff,
                        current_heading,
                    )
                    logger.info(
                        "Yaw aligned within 5° (heading %.1f°) – rotation complete",
                        current_heading,
                    )
                    return
                time.sleep(1.0 / self.rate_hz)
                continue  # wait for next check
            else:
                first_success_check = None  # reset success check

            koef = diff // 20 * minimum_sensitivity
            yaw_power = 1500 + yaw_direction * (
                minimum_sensitivity + koef + int(diff / 2)
            )
            logger.debug(
                "Yaw ctrl: target=%.1f° current=%.1f° diff=%.1f° pwm=%d",
                angle,
                current_heading,
                diff,
                yaw_power,
            )
            self.change_radio_control(
                yaw=yaw_power,
            )
            time.sleep(1.0 / self.rate_hz)
            if time.time() - start > timeout_s:
                logger.error("Rotation timed‑out after %s s", timeout_s)
                raise TimeoutError("Rotation exceeded time limit")

    def rotate_to_target(self, target: Coordinates):
        logger.info(
            "Rotating towards target (%.6f, %.6f)…", target.latitude, target.longitude
        )
        minimum_sensitivity = 20  # minimal yaw sensitivity
        while True:
            pos = self.get_position()
            target_bearing = self.angle(
                pos.coord.latitude,
                pos.coord.longitude,
                target.latitude,
                target.longitude,
            )
            current_heading = pos.heading
            diff = (target_bearing - current_heading + 540) % 360 - 180
            yaw_direction = -1 if diff < 0 else 1
            diff = abs(diff)
            if diff < 5:
                logger.info(
                    "Yaw aligned within 5° (heading %.1f°) – rotation complete",
                    current_heading,
                )
                return
            koef = diff // 20 * minimum_sensitivity
            yaw_power = 1500 + yaw_direction * (
                minimum_sensitivity + koef + int(diff / 2)
            )
            logger.debug(
                "Yaw ctrl: target=%.1f° current=%.1f° diff=%.1f° pwm=%d",
                target_bearing,
                current_heading,
                diff,
                yaw_power,
            )
            self.change_radio_control(
                yaw=yaw_power,
            )
            time.sleep(1.0 / self.rate_hz)

    # ---------------------------------------------------------------------
    # Navigation helpers
    # ---------------------------------------------------------------------
    def distance_to_target(
        self, target_lattitude: float, target_longitude: float
    ) -> float:
        """Distance from current location to *target* (metres)."""
        position = self.get_position()
        return self.calculate_distance(
            position.coord.latitude,
            position.coord.longitude,
            target_lattitude,
            target_longitude,
        )

    def goto_waypoint(
        self,
        target: Coordinates,
        timeout_s: int = 600,
        min_distance: float = 0.5,
    ):
        """Move (guided) to *target* within ``timeout_s`` seconds."""
        logger.info(
            "Navigating to waypoint (%.6f, %.6f) – timeout=%ds, tolerance=%.2fm",
            target.latitude,
            target.longitude,
            timeout_s,
            min_distance,
        )
        start = time.time()
        while True:
            distance = self.distance_to_target(target.latitude, target.longitude)
            logger.debug("Waypoint nav: distance=%.2f m", distance)
            if distance < min_distance:
                logger.info("Reached waypoint within %.2f m", distance)
                return
            if time.time() - start > timeout_s:
                logger.error("Navigation timed‑out after %s s", timeout_s)
                raise TimeoutError("Moving to the target exceeded the time limit.")

            self.rotate_to_target(target)
            self.change_radio_control(pitch=1000)  # pitch forward
            time.sleep(1.0 / self.rate_hz)

    # ---------------------------------------------------------------------
    # Safety helpers
    # ---------------------------------------------------------------------
    def force_disarm(self):
        logger.warning("Force‑disarming vehicle…")
        msg = self.vehicle.message_factory.command_long_encode(
            0,
            0,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            21196,
            0,
            0,
            0,
            0,
            0,
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        logger.debug("Disarm command sent")
