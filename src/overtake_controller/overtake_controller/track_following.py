"""Track reference for the simulated overtaker between passing maneuvers."""

import math
import numpy as np

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

    def protect(self, node, steering, speed, previous_steer):
        """Reject commands whose braking rollout puts a footprint into a scan return."""
        scan = node.scan_msg
        if scan is None or not scan.ranges or scan.angle_increment <= 0:
            return 0.0, 0.0
        ranges = np.nan_to_num(np.asarray(scan.ranges, dtype=float),
                               nan=0.0, posinf=scan.range_max, neginf=0.0)
        low = max(-node.MAX_STEER_ANGLE, previous_steer - node.MAX_STEER_DELTA)
        high = min(node.MAX_STEER_ANGLE, previous_steer + node.MAX_STEER_DELTA)
        track_steer, _ = self.control(node.opp_x, node.opp_y, node.opp_yaw,
                                      node.wheelbase, node.MAX_SPEED)
        turns = list(dict.fromkeys(float(np.clip(value, low, high))
                                   for value in (steering, track_steer, low, high, previous_steer)))
        speeds = list(dict.fromkeys(max(0.0, min(speed, value)) for value in (speed, 1.4, 0.9, 0.4, 0.0)))

        def clear(turn, target):
            x = y = yaw = 0.0
            velocity = node.opp_speed
            corners = []
            for _ in range(8):
                velocity = max(target, velocity - 0.4) if velocity > target else min(target, velocity + 0.3)
                yaw += velocity * math.tan(turn) * 0.1 / node.wheelbase
                x += velocity * math.cos(yaw) * 0.1
                y += velocity * math.sin(yaw) * 0.1
                for front in (-0.29, 0.29):
                    for side in (-0.155, 0.155):
                        corners.append((x + front * math.cos(yaw) - side * math.sin(yaw) - 0.275,
                                        y + front * math.sin(yaw) + side * math.cos(yaw)))
            points = np.asarray(corners)
            angles = np.arctan2(points[:, 1], points[:, 0])
            indices = np.rint((angles - scan.angle_min) / scan.angle_increment).astype(int)
            visible = (indices >= 0) & (indices < len(ranges))
            distances = np.linalg.norm(points, axis=1)
            return bool(np.all(distances[visible] + 0.12 < ranges[indices[visible]]))

        for candidate_speed in speeds:
            for turn in turns:
                if clear(turn, candidate_speed):
                    return turn, candidate_speed
        return float(np.clip(track_steer, low, high)), 0.0
