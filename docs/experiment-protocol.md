# Scenario experiment protocol

The normal `start_f1tenth.ps1` launcher uses the positions and yaws already in
`config/manager.yaml`. Each scenario now also specifies `ego_speed` and
`opp_speed`, in m/s. The opponent's configured speed must exceed the ego's.

The example speed pairs repeat across the 24 starts:

| Scenarios | Ego speed | Opponent speed |
|---|---:|---:|
| 1, 4, 7, 10, 13, 16, 19, 22 | 0.8 | 1.2 |
| 2, 5, 8, 11, 14, 17, 20, 23 | 1.0 | 1.4 |
| 3, 6, 9, 12, 15, 18, 21, 24 | 1.2 | 1.6 |

These are configurable example speeds; no experimental speed table was supplied.
Starting yaws remain radians. No starting positions or yaws were replaced.

## Reset and initialization

1. The manager waits for bridge observations before the first physical reset.
2. `SETTLING` physically resets both poses and clears controls, estimators and
   planner state. The configured two-second reset hold rejects old observations.
3. `WARMUP` applies the configured speeds to the actual simulator vehicle states.
   Both cars follow the track, while defense and passing plans remain disabled.
   The defensive IMM receives opponent odometry through `sim_opp_frenet.py`.
4. The IMM acknowledges at least `imm_initialization_seconds` (minimum 1.5) of
   continuous fresh observations, including enough samples at 20 Hz. Observation
   gaps restart this initialization window. Each acknowledgment includes the
   reset timestamp, so a previous scenario cannot release the next one.
5. The manager enters `RUNNING` once that scenario's IMM is ready and the opponent
   remains behind. Defense and passing are enabled. Missing initialization or an
   invalid starting order produces an `ERROR` and stops the session.
6. The experiment finishes when the opponent passes by `overtake_margin`, or the
   ego reaches the configured lap limit. Both cars stop before the next reset;
   the final scenario shuts down the pipeline.

Warmup is moving observation time, separate from the stationary reset hold.
Its samples are included in the CSV and marked `WARMUP`. Filter on
`experiment_phase == RUNNING` to analyze active defensive/passing behavior.
The defensive estimator continues receiving simulator opponent odometry after
initialization; this remains a ground-truth-assisted simulation experiment.

Speed values are initial model velocities and controller references/ceilings.
Steering, following distance and collision avoidance may reduce actual speed.
Safety braking can stop the opponent; the code does not force it to remain faster
than the ego in every measured sample. `opponent_faster` and `opp_speed_limited`
make those deviations visible. A study requiring strictly greater measured
opponent speed throughout must classify these deviations explicitly.

## Results

Each session retains `simulation_1.csv` through `simulation_24.csv`, plus a
snapshot of `manager_settings.yaml` and `opp_overtake_settings.yaml`.
`scenario_events.jsonl` records phase changes, timestamps, configured poses and
speeds, and final completion/error reasons. The event sidecar is authoritative
for trial outcomes.

The original CSV columns remain, followed by scenario/phase, configured speeds,
actual yaws, requested speeds, current and latched collision flags, the separate
defensive IMM prediction/readiness/window, continuous relative track progress,
and measured speed-order flags. `imm_trajectory` keeps its original meaning: the
overtaker's ego prediction. `defensive_imm_trajectory` is the defender's opponent
prediction.

The logger truncates snapshots at a stamped reset or completion boundary even
when the event arrives after new odometry. Manual removal of final reset rows
should not be necessary for new runs. Each car's track progress is unwrapped
independently, preventing a stopped opponent from becoming an apparent leader
when the ego travels half a track ahead. Lap detection also requires progress
around the track instead of merely leaving and re-entering the spawn bubble.

The old `saved_results/natural24_20261102_154237` data predates these changes.
Its existing review and cleaned copies remain applicable to that older run.

## Verification commands

Inside the built image, run `bash /sim_ws/src/f1tenth_gym_ros/test/run_protocol_checks.sh`.
For real driving verification, use:

```bash
bash /sim_ws/src/f1tenth_gym_ros/test/run_protocol_live.sh \
  --output /verification/new_run --scenarios 3 --controller overtake --timeout 400
```

Use a fresh output directory for each verification. `overtake_aggressive` selects
the aggressive opponent. The live check uses natural driving and completion
events, and audits warmup, actual initial speeds, defensive activation, CSV
boundaries, and stopped odometry. Three-scenario checks do not establish complete
24-scenario endurance coverage or collision avoidance in all starting situations.

## Verification recorded on November 2, 2026

| Check | Result |
|---|---|
| Unit/regression suites | 36 passed: 5 combined workflow, 12 lifecycle/configuration, 9 driving, 10 experiment protocol. |
| Regular opponent, natural scenarios 1-3 | All passed with defense activation and real overtakes; zero observed collision flags. |
| Aggressive opponent, natural scenarios 1-3 | All passed with defense activation and real overtakes; zero observed collision flags. |
| Actual initial velocities | All six natural trials first reported the configured pairs exactly: 0.8/1.2, 1.0/1.4, and 1.2/1.6 m/s. |
| Initialization | All six natural trials waited 1.511-1.515 seconds; no defense state occurred during warmup. |
| Natural CSV boundaries | No reset jumps or malformed rows in either controller's three-scenario sequence. |
| Natural final shutdown | Both cars' measured odometry speeds reached 0.0 m/s for both controller variants. |
| All 24 starting pairs | Controlled workflow passed in 131.9 seconds; exact spawns, driving resumption, initialization, 24 nonempty CSVs, and final shutdown. |

The 24-start workflow used synthetic completion inputs delivered to an isolated
manager. Its CSVs are verification samples, not 24 natural-race outcomes. The
normal launcher continues to use real position/odometry inputs. A full natural
24-scenario endurance run with this new initialization/speed protocol has not
been performed.

Local verification reports and logs are archived under
`saved_results/protocol_verification_20261102/`. The sibling `branch_review/`
directory retains the intermediate development checks.
