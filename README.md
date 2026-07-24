# F1Tenth Parallel Simulation Manager
> [!IMPORTANT]
>This utilizes the `f1_gym_ros` containerized Docker simulator, for installation instructions and configuring the simulator please visit its [repository](https://github.com/f1tenth/f1tenth_gym_ros) and follow its documentation.

Run the file `.\start_f1tenth.ps1` to start the parallel simulation process. This process can take **a long time to build**, it builds the docker containers and binds their noVNC viewpoints while launching the relevant commands.

Once finished, navigate to the **dashboard** to view all screens by clicking `dashboard.html` in the file tree or clicking the link provided in the terminal once the launch command is finished. 

In the dashboard, you can click `FULL` to go to that session's full screen view. 

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
