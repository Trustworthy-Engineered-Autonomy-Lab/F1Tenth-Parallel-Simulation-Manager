# F1Tenth Parallel Simulation Manager
> [!IMPORTANT]
>This utilizes the `f1_gym_ros` containerized Docker simulator, for installation instructions and configuring the simulator please visit its [repository](https://github.com/f1tenth/f1tenth_gym_ros) and follow its documentation.

> [!NOTE]
> To kill all Docker containers after simulation if termination does not proceed properly use:
> ```shell
> docker ps -a -q --filter "name=session" | ForEach-Object { docker rm -f $_ }
> ```

Run the bash file `.\start_f1tenth.ps1` to start the parallel simulation process. This process can take **a long time to build**, it builds the docker containers and binds their noVNC viewpoints while launching the relevant commands.

Once finished, navigate to the **dashboard** to view all screens by clicking `dashboard.html` in the file tree or clicking the link provided in the terminal once the launch command is finished. 

In the dashboard, you can click `FULL` to go to that session's full screen view. 
