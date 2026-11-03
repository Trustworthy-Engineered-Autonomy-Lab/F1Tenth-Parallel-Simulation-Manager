"""Shared experiment configuration and continuous track-progress accounting."""
import math


def protocol_qos():
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE)


def scenario_speeds(scenario):
    # Legacy configurations retain a conservative default speed pair.
    ego = scenario.get('ego_speed', 1.0)
    opp = scenario.get('opp_speed', 1.4)
    for value in (ego, opp):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('Scenario speeds must be positive finite numbers in m/s')
    if opp <= ego:
        raise ValueError('opp_speed must be greater than ego_speed')
    return float(ego), float(opp)


class TrackProgress:
    """Unwrap each car independently so a half-track separation cannot swap leaders."""
    def __init__(self, length):
        self.length = length
        self.last = {}
        self.distance = {'ego': 0.0, 'opp': 0.0}
        self.initial_gap = None

    def wrap(self, value):
        return (value + self.length / 2) % self.length - self.length / 2

    def update(self, car, position):
        if car in self.last:
            self.distance[car] += self.wrap(position - self.last[car])
        self.last[car] = position
        if self.initial_gap is None and len(self.last) == 2:
            self.initial_gap = self.wrap(self.last['opp'] - self.last['ego'])
            self.distance = {'ego': 0.0, 'opp': 0.0}

    @property
    def gap(self):
        if self.initial_gap is None:
            return None
        return self.initial_gap + self.distance['opp'] - self.distance['ego']
