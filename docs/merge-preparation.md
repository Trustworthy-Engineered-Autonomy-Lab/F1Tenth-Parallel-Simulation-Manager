# Defensive branch and scenario-manager compatibility

Prepared against `feature/defensive-maneuvering` at `40abe1a` and remote `main` at
`e2de435`. No merge, commit, or push was performed during this preparation.

The common ancestor is `838979e`. The scenario manager was added on main after
the branches split; the defensive branch still contains the earlier subscriber-only
manager. Comparing the two final files alone can misleadingly look like a removal.

## What to retain when a merge is approved

| Area | Resolution |
| --- | --- |
| `config/manager.yaml` | Keep main's scenario configuration, including its 24 starting pairs, lap settings, and overtake margin. |
| `src/f1tenth_env_manager/env_manager/manager.py` | Keep main's manager: initial spawning, lap/overtake transitions, scenario numbering, settle time, and session completion. |
| `f1tenth_gym_ros/gym_bridge.py` | Retain main's `PoseArray` subscription and atomic two-car reset handler alongside the defensive branch's collision publishers and collision-state updates. |
| `src/data_logger/data_logger/logger.py` | Preserve main's per-scenario CSV rotation and reset/shutdown handling. Preserve the defensive branch's IMM trajectory/status columns when adapting its telemetry; keep CSV headers and row fields aligned. |
| `auto_run_sim.sh` | Keep the defensive branch's complete `maneuver_launch.py` pipeline and its controller-profile selection, plus the manager configuration environment variables. Do not launch `ego_ftg` alongside dynamic MPPI: both would publish `/drive`. |
| `launch/maneuver_launch.py` | Keep the defensive pipeline and its single manager/logger instance. After the merge those executable names will load main's restored manager/logger implementations. |
| `start_f1tenth.ps1` and `start_f1tenth.sh` | Keep YAML-driven manager settings and explicit overrides, with error checking and configuration validation before existing sessions are removed. The prepared code uses the image's PyYAML rather than adding a host YAML module dependency. |
| Defensive and overtaking controllers | Keep the defensive branch's driving fixes and the new scenario lifecycle handling. |
| `src/lidar_processing` | Keep the defensive branch's vendored package and its Python modules, not main's older gitlink entry. |
| `config/opp_overtake_settings.yaml` | Keep implemented profiles: regular, aggressive, regular. The old left/right/conservative entry points do not have corresponding modules. |
| Generated `results/` files | Preserve both sets as experiment artifacts; avoid overwriting them while resolving source changes. Some are tracked despite the ignore rule. |

Five files changed on both branches since their common ancestor: the startup shell
script, bridge, manager, logger, and PowerShell launcher. These need careful review
even if Git can combine some edits automatically.

## Preparation already implemented on the defensive branch

- Shared lifecycle handling subscribes to `/env_manager/reset_scenario` (`PoseArray`)
  and `/env_manager/shutdown` (`Bool`). The paired pose message is used as the reset
  event so the manager's separate logging-reset notification does not reset a node twice.
- A respawn clears MPPI's active divert, warm start, and synchronized sensor queues;
  clears committed passing paths and steering memory; resets both IMM estimators;
  and invalidates the opponent-observation timer and defensive path converter.
- Controllers stop during the reset settle time, require fresh localization/scans,
  and reject stamped inputs from before that interval. Shutdown stops both cars and
  prevents further commands.
- A reset invalidates any RRT planning cycle already running. Publication is
  serialized with marker clearing so an old scenario's result cannot reappear.
- Launchers pass `NUM_SIMULATIONS`, `MAX_LAPS`, `OVERTAKE_MARGIN`, `CONFIG_FILE`, and
  `SCENARIO_RESET_HOLD_SECONDS`. YAML is optional until main's config is brought in;
  explicit PowerShell options or shell environment settings override YAML.

With main's current YAML, defaults are **one parallel Docker session, 24 scenarios,
one lap per scenario, and a 3 m overtake margin**. These are separate concepts. Use
`-NumSessions 3` explicitly if three parallel sessions are wanted after the merge.

The working branch still uses its subscriber-only manager. These changes prepare
the defensive nodes for main's manager; they do not activate main's reset system
before the actual merge is approved.

## Validation

- All seven ROS packages build in a separate `f1tenth_merge_review` image.
- Six isolated checks passed against unchanged main manager/logger/bridge snapshots:
  spawn pairs, lap counting, overtake-margin transitions, completion/shutdown,
  CSV rotation, and atomic physical reset with cleared commands.
- Twelve new reset/configuration checks and eight existing driving regressions pass.
- A live isolated check ran main's unchanged bridge snapshot with the prepared
  defensive pipeline. Scenarios `s_1` and `s_2` each spawned both cars at exactly
  the configured positions, stopped them during the hold, and resumed both with
  fresh observations. Shutdown stopped both cars.
- A mocked PowerShell launcher check verified main-style YAML settings and explicit
  overrides without starting or deleting ordinary simulation containers.
- Shell syntax, PowerShell parsing, and diff-whitespace checks passed.

The live compatibility check covers two respawns and shutdown; it is not a claim
that all 24 scenarios are collision-free. The complete merged manager-to-controller
run should be checked once integration is approved, including final session completion
and per-scenario logs.

The snapshot archives and isolated check drivers are kept in the sibling
`../branch_review/` directory, outside the working repository.
