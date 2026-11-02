"""Track reference for the simulated overtaker between passing maneuvers."""

import math

from racetrack_utilities.racetrack_utilities import RacetrackUtilities


class RacelineFollower:
    def __init__(self, csv='/sim_ws/src/lidar_processing/scripts/Spielberg_map.csv'):
        self.track = RacetrackUtilities(csv)

    def control(self, x, y, yaw, wheelbase, max_speed):
        s, d = self.track.convert_to_frenet(x, y)
        target = self.track.convert_to_cartesian(s + 1.2, 0.0)
        dx, dy = target - [x, y]
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
        steer = math.atan2(2.0 * wheelbase * local_y, dx * dx + dy * dy)
        speed = max_speed / (1.0 + 3.0 * abs(steer))
        if abs(d) > 0.5:
            speed = min(speed, 1.2)
        return steer, speed
