"""Reset/shutdown contract shared by the driving nodes and the environment manager."""

from functools import wraps
from threading import RLock
import os
import json

from geometry_msgs.msg import PoseArray
from std_msgs.msg import Bool, String
from lidar_processing.experiment_protocol import protocol_qos


class ScenarioLifecycle:
    def __init__(self, node, on_reset, callbacks=(), callback_group=None,
                 allowed_phases=('WARMUP', 'RUNNING')):
        self.node = node
        self.on_reset = on_reset
        self.lock = RLock()
        self.generation = 0
        self.resume_at_ns = 0
        self.stopped = False
        self.last_reset_stamp_ns = 0
        self.phase = 'RUNNING'  # unmanaged/debug launches keep their existing behavior
        self.configuration = None
        self.allowed_phases = allowed_phases
        self.state_sub = node.create_subscription(
            String, '/env_manager/experiment_state', self.experiment_state, protocol_qos(),
            callback_group=callback_group)
        node.declare_parameter('scenario_reset_hold_seconds',
                               float(os.getenv('SCENARIO_RESET_HOLD_SECONDS', '2.0')))
        self.hold_seconds = float(node.get_parameter('scenario_reset_hold_seconds').value)
        if self.hold_seconds < 0:
            raise ValueError('scenario_reset_hold_seconds must be nonnegative')
        for name in callbacks:
            setattr(node, name, self.guard(getattr(node, name)))
        self.reset_sub = node.create_subscription(
            PoseArray, '/env_manager/reset_scenario', self.reset, 10,
            callback_group=callback_group)
        self.shutdown_sub = node.create_subscription(
            Bool, '/env_manager/shutdown', self.shutdown, 10,
            callback_group=callback_group)

    @property
    def ready(self):
        return (not self.stopped and self.phase in self.allowed_phases
                and self.node.get_clock().now().nanoseconds >= self.resume_at_ns)

    def experiment_state(self, message):
        state = json.loads(message.data)
        if self.configuration and state['reset_stamp_ns'] < self.configuration['reset_stamp_ns']:
            return
        new_scenario = not self.configuration or state['reset_stamp_ns'] != self.configuration['reset_stamp_ns']
        self.configuration = state
        self.phase = state['phase']
        if new_scenario and hasattr(self.node, 'configure_experiment'):
            self.node.configure_experiment(state)

    def accepts(self, message=None):
        if not self.ready:
            return False
        header = getattr(message, 'header', None)
        if header is None and getattr(message, 'markers', None):
            header = message.markers[0].header
        if header is not None and self.resume_at_ns:
            stamp_ns = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
            if stamp_ns < self.resume_at_ns:
                return False
        return True

    def guard(self, callback):
        @wraps(callback)
        def guarded(*args, **kwargs):
            if self.accepts(args[0] if args else None):
                return callback(*args, **kwargs)
            return None
        return guarded

    def reset(self, message):
        if len(message.poses) != 2 or self.stopped:
            return
        with self.lock:
            stamp = message.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            if stamp_ns and stamp_ns <= self.last_reset_stamp_ns:
                return
            self.last_reset_stamp_ns = stamp_ns
            self.generation += 1
            self.resume_at_ns = (self.node.get_clock().now().nanoseconds
                                 + int(self.hold_seconds * 1e9))
            self.on_reset(message)

    def shutdown(self, message):
        if message.data and not self.stopped:
            with self.lock:
                self.stopped = True
                self.generation += 1
                self.on_reset(None)
