# python-sim
JBSim (but it is python)

## Setup

1. Get PX4-Autopilot from https://github.com/PX4/PX4-Autopilot.git
2. Build PX4 in Tools/setup/ubuntu.sh
3. Open ports 4560 and 14540
4. Open wsl terminal in PX4 folder
5. run `export PX4_SIM_HOST_ADDR=172.28.112.1` (use ipconfig in Windows to recover WSL ip on local machine)
6. First, in python-sim/src run `py mavlink_connector.py`
7. Then, run `make px4_sitl none_iris`

This should connect the simulator to the python file. 