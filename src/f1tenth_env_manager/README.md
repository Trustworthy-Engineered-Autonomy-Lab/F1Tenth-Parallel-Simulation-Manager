# F1TENTH Environment Manager

An environment manager for F1TENTH simulator (f1tenth_gym_ros) with an automated script workflow to launch an environment utilizing multiple vehicles.

## Script Automation
See `scripts/README.md` for detailed instructions for how to use the automated script to get the envioronment running. In a typical workflow you will only use the script which will...
- start Docker
- wait for the container
- enter the container
- start a tmux session
- build the workspace
- launch the simulator
- launch FTG
- launch the manager

### Manager.py
manager.py is a file for tracking how many laps both the opponent and ego complete and how many of those laps the ego was ahead. It also logs distance between the cars, speed of each cars, and the x and y of each car. There is also a reset function that can reset the cars back to their original position by typing r in the terminal.

### Sim Follow the Gap
Sim follow the gap is a FTG that runs both cars at once for automation and simplicity. You can also only run one car and run another algorithm on the ego vehicle for testing.
