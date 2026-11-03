# Combined defensive driving and 24-scenario runner

This records the original merge verification. The later experiment-protocol
changes and their verification are documented in [experiment-protocol.md](experiment-protocol.md).

Integrated the feature branch at `584e494` with main at `e2de435` on 2026-11-02.
Both histories and their recorded experiment outputs are retained.

## Behavior preserved

- Main's `manager.yaml`: one Docker session, 24 configured starting pairs, one lap
  maximum per scenario, and a 3 m overtake margin. Explicit launcher options can
  override the session/lap/scenario counts and margin.
- Main's spawning, lap/overtake transitions, scenario numbering, shutdown, and
  `simulation_1.csv` through `simulation_24.csv` output.
- The feature branch's dynamic MPPI defense, RRT*/Stackelberg planning, opponent
  overtaking, collision telemetry, and reset-aware state clearing.
- Scenario CSVs retain the IMM trajectory/status columns and include MPPI state
  and opponent driving mode. Rows wait for fresh post-reset observations.
- Initial spawning waits for bridge observations, preventing compilation delays or
  the default simulator pose from arming a false initial lap. Final shutdown stops
  the bridge and rejects late drive commands.
- Both opponent profiles check a braking footprint rollout against current scan
  returns before publishing a command. Insufficient clearance reduces speed or brakes.

## Checks performed

| Check | Result |
| --- | --- |
| Build | All seven ROS packages build in the simulator image. |
| Unit/regression checks | 26 passed: five combined-workflow checks, 12 lifecycle/config checks, nine driving regressions. |
| All configured starts | All 24 paired respawns, driving resumption, CSV schemas/rows, and final shutdown passed in 117.8 seconds. |
| Regular opponent encounter | 60 seconds; actual ego defensive shift about 0.39 m, opponent passed, zero collision flags or out-of-track samples. |
| Aggressive opponent encounter | 60 seconds; actual ego defensive shift about 0.31 m, opponent passed, zero collision flags or out-of-track samples. |

The all-starts check used `--controlled-transitions`: synthetic lap/overtake
completion inputs were delivered only to an isolated manager, while the simulator
and driving controllers ran normally at every configured starting pair. This
verifies orchestration and file output, not complete natural racing performance
across all 24 scenarios. A full natural 24-scenario endurance run remains unmeasured.

Reproduce the workflow check inside a sourced container with:

```bash
python3 /sim_ws/src/f1tenth_gym_ros/test/check_scenario_workflow.py --controlled-transitions --output /tmp/workflow-check
```

Omit `--controlled-transitions` to wait for real laps/overtakes through the complete
sequence. The regular launcher always uses the real manager inputs.

Detailed local reports and isolated CSVs from this integration are in the sibling
`../branch_review/combined_24_verified/`, `combined_driving_standard/`, and
`combined_driving_aggressive/` directories. Production session results were not
overwritten by verification.
