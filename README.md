# F1Tenth Parallel Simulation Manager
> [!IMPORTANT]
>This utilizes the `f1_gym_ros` containerized Docker simulator, for installation instructions and configuring the simulator please visit its [repository](https://github.com/f1tenth/f1tenth_gym_ros) and follow its documentation.

Run the file `.\start_f1tenth.ps1` to start the parallel simulation process. This process can take **a long time to build**, it builds the docker containers and binds their noVNC viewpoints while launching the relevant commands.

Once finished, navigate to the **dashboard** to view all screens by clicking `dashboard.html` in the file tree or clicking the link provided in the terminal once the launch command is finished. 

In the dashboard, you can click `FULL` to go to that session's full screen view. 

### Defensive branch

The ego car follows dynamic MPPI on `/drive`. The opponent follows the track on
`/opp_drive` until the overtaking planner supplies `/overtake_spline`, then tracks
that path. The ego's separate Frenet IMM feeds the RRT*/Stackelberg defensive
planner, whose chosen spline reaches MPPI through the path converter. Its
prediction uses `/defensive_imm_path`; the opponent's ego prediction uses `/imm_path`.

Overtaking paths follow the known simulator track and check the car's footprint
against its boundaries. Planning starts within 6 m (`trigger_distance` on
`overtake_interceptor`), including the default spawn gap. Pursuit lookahead uses
actual path distance, and completed paths are released instead of tracked forever.
The remaining passing path is rechecked against the moving defender. If blocked,
the planner tries the other side, then falls back to track following and retries
on later planning cycles. A following-distance speed limit applies when the ego
enters the opponent's lane. The legacy status name `FTG` denotes this fallback;
the normal fallback now uses a raceline reference to avoid gap-control corner clips.
Before sending a drive command, both opponent profiles also check a braking rollout
of the car's footprint against the latest scan. They reduce speed or brake when
the proposed command has insufficient clearance.
Dynamic MPPI accepts updated defensive paths while maneuvering (`allow_interrupt:
True`); otherwise an initial hold-line choice can suppress a later blocking move.
Hold-line choices do not replace active blocking paths. Defensive lane changes
reach their target over 2.2 m, and MPPI samples 256 trajectories per control update.
The RRT* budget is 20 ms per sampled start (`rrt_time_budget`), rather than 200 ms,
so a five-start defensive cycle can react before the opponent passes.

On Windows, Docker Desktop must be running with Linux containers. ROS and Python
dependencies are installed in the image; no host Python or CUDA installation is
needed. Start with one session from PowerShell:

```powershell
.\start_f1tenth.ps1 -NumSessions 1
```

If PowerShell reports that running scripts is disabled, use a process-only
override (this does not change the machine's execution policy):

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\start_f1tenth.ps1
```

That command starts all three sessions; append `-NumSessions 1` for one session.

Open [session 1](http://localhost:8081/vnc.html). `config/manager.yaml` defaults to
one Docker session running 24 scenarios, with one lap maximum per scenario and a
reset when the opponent is 3 m ahead. Use `-NumSessions 3` to run three parallel
sessions with regular, aggressive, and regular opponent profiles. Only `overtake` and
`overtake_aggressive` are implemented in `config/opp_overtake_settings.yaml`.

The complete behavior pipeline is defined in `launch/maneuver_launch.py`. A node
exiting stops that pipeline and reports the failure. Inspect behavior logs with:

```powershell
Get-Content .\results\session_1\maneuvers.log -Tail 60 -Wait
```

Look for `defensive lane selected`, `DIVERT accepted`, `RETURN`, `RACELINE`, and
`OVERTAKE_ACTIVE`. These confirm planner/controller transitions; moving cars alone
do not confirm either maneuver. Defensive lane selection can legitimately choose
to hold the current line, and passing depends on available space.

The image builds all seven ROS packages. To run focused regressions and a headless
behavior check without changing collected session results:

```powershell
docker run --rm f1tenth_gym_ros -lc 'source /opt/ros/foxy/setup.bash && source /sim_ws/install/setup.bash && python3 /sim_ws/src/f1tenth_gym_ros/test/test_maneuver_regressions.py'
docker run --rm f1tenth_gym_ros -lc 'source /opt/ros/foxy/setup.bash && source /sim_ws/install/setup.bash && python3 /sim_ws/src/f1tenth_gym_ros/test/check_maneuvers.py --duration 90'
```

The single-encounter behavior check disables the scenario manager so it can measure
continuous driving without respawns. It requires both cars to move, a lateral defensive path, actual
ego lateral movement while in `DIVERT`, a nonempty overtaking path, and the
opponent moving from behind to ahead, with both car centres inside the track and
zero simulator collision flags. Collision flags are published on
`/ego_racecar/collision` and `/opp_racecar/collision` (both `std_msgs/Bool`).
Add `--controller overtake_aggressive` for
the second profile. Reports and logs are written under `/tmp/maneuver-check` in
the test container. The first simulation starts more slowly while Numba compiles.

For the complete configured scenario sequence, run
`test/check_scenario_workflow.py` in the sourced container. It checks all configured
spawns, scenario CSVs, defensive telemetry columns, and final shutdown. The normal
mode waits for actual overtakes/laps. Add `--controlled-transitions` for a quick
workflow check using synthetic completion events on isolated manager inputs;
that mode does not measure full-race performance or prove collision-free racing.

The manager waits for the bridge to publish observations before its first spawn.
Controllers clear old plans and filter state at each reset; the logger waits for
fresh observations before writing rows to the new scenario's CSV. Final shutdown
also stops the bridge and ignores late drive commands.

### Important Commands
- **Build the simulator (when making any changes or on first install)**
```powershell
docker build -t f1tenth_gym_ros .
```

- **Start all the parallel environments**
```powershell
.\start_f1tenth.ps1
```
> On linux use `start_f1tenth.sh`

>[!IMPORTANT]
>If the Rviz terminal does **not** launch, make sure that the `auto_run_sim.sh` is **LF** and not **CRLF** (change it and rebuild)

- **Check general docker logs**
```powershell
docker logs -f f1tenth_session_1
```

- **Check the tmux panels in the first parallel session**

To get into the container
```powershell
docker exec -it f1tenth_session_1 bash
```
When in the container (`root@x:/sim_ws#`):
```bash
tmux attach -t f1sim_1
```
- **To kill all containers after collecting data**
```shell
docker ps -a -q --filter "name=session" | ForEach-Object { docker rm -f $_ }
```
> On linux use,
> ```bash
> docker kill $(docker ps -q)
> ```

---
### Folders and Files
- `config/sim.yaml` - Configuration file for all 3 parallel sims
- `src/` - Contains all the source files for the simulator, any changes here will be reflected in the simulator after a rebuild.
  - Important packages/nodes:
    - `data_logger` manages results
    - `f1tenth_env_manager` managed **each** parallel env
    - `overtake_controller` is the overtake manager including imm filter and the slow ftg
    - `frenet_frame_conv` converts cartesian coords to frenet frames (s,d)
- `results/` - This contains the most recent simulation results.
  - **NOTE**: When launching the sim again, everything in this file will be **REPLACED**. So 'save' results by copying them over to the `saved_results/` folder.
  - Results are managed by the `data_logger` package. They are seperated into different iterations based on when an overtake +5m occurs, then the cars are reset to random positions.
- `saved_results/` - Store all saved results here. Use the format of `<name>_<flag>_<num>`. Contains previously saved results.


---
#### todo changes
- make rviz fullscreen upon launch
- make robotmodel for opp show up automatically
- ensure the data collection works (idt it does rn)
- make the termination condition actually terminate the simulation
  - includes ctrl c all panes or just killing all relevant docker containers 
