"""Read manager.yaml for either launcher using the image's existing PyYAML."""

import argparse
import json
import math
from pathlib import Path

import yaml


DEFAULTS = {'num_sessions': 3, 'max_laps': 3, 'num_simulations': 3,
            'overtake_margin': 3.0, 'reset_settle_seconds': 2.0}


def read_config(path):
    path = Path(path)
    raw = yaml.safe_load(path.read_text(encoding='utf-8')) if path.exists() else {}
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise ValueError('manager.yaml must contain a mapping')
    result = dict(DEFAULTS)
    for name, default in DEFAULTS.items():
        value = raw.get(name, default)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{name} must be a finite number')
        if isinstance(default, int):
            if int(value) != value or value < 1:
                raise ValueError(f'{name} must be a positive integer')
            value = int(value)
        elif value < 0 or (name == 'overtake_margin' and value == 0):
            raise ValueError(f'{name} is outside its valid range')
        result[name] = value
    if result['num_sessions'] > 100:
        raise ValueError('num_sessions must not exceed 100')
    warmup = raw.get('imm_initialization_seconds', 1.5)
    if isinstance(warmup, bool) or not isinstance(warmup, (int, float)) or not math.isfinite(warmup) or warmup < 1.5:
        raise ValueError('imm_initialization_seconds must be at least 1.5')
    try:
        from lidar_processing.experiment_protocol import scenario_speeds
    except ModuleNotFoundError:
        from experiment_protocol import scenario_speeds  # launcher's standalone script entry point
    scenarios = raw.get('sim_start', {})
    if not isinstance(scenarios, dict):
        raise ValueError('sim_start must be a mapping')
    if scenarios and any(f's_{i}' not in scenarios for i in range(1, result['num_simulations'] + 1)):
        raise ValueError('Each requested simulation must have a sim_start entry')
    for scenario in scenarios.values():
        if not isinstance(scenario, dict):
            raise ValueError('Each scenario must be a mapping')
        for car in ('ego', 'opp'):
            pose = scenario.get(car)
            if (not isinstance(pose, list) or len(pose) != 3 or any(
                    isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in pose)):
                raise ValueError(f'{car} must contain finite [x, y, yaw_radians]')
        scenario_speeds(scenario)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('path')
    parser.add_argument('--shell', action='store_true')
    args = parser.parse_args()
    config = read_config(args.path)
    if args.shell:
        print(' '.join(str(config[key]) for key in DEFAULTS))
    else:
        print(json.dumps(config))


if __name__ == '__main__':
    main()
