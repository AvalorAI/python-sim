# python-sim
JBSim (but it is python)

## Setup

1. Get PX4-Autopilot from https://github.com/PX4/PX4-Autopilot.git
2. Open ports 4560 and 14540
3. Open wsl terminal in PX4 folder
4. run `export PX4_SIM_HOST_ADDR=172.28.112.1` (use ipconfig in Windows to recover WSL ip on local machine)
5. run `make px4_sitl none_iris`
6. In python-sim/src run `py mavlink_connector.py`

This should connect the simulator to the python file. 