# Baxter Control Environments
`gym-baxter` provides an interface for performing reinforcement learning and control experiments on robots in simulation and on real hardware. `gym-baxter` uses PyBullet as the backend for simulation experiments and ROS as the backend for hardware experiments. `gym-baxter` also makes use of the [OpenAI Gym](https://gym.openai.com/) paradigm that is common for reinforcement learning environments.

## Dependencies
Python 2.7 \
PyBullet \
OpenAI Gym \
ROS Kinetic \
Baxter SDK \
PyKDL \
baxter_pykdl

## Install
Install the Python dependencies:
```
pip install pybullet gym
```

To install gym-robotics:
```
git clone https://github.com/jmichaux/gym-baxter
cd gym-baxter
pip install -e .
```

To install ROS, follow the directions [here](http://wiki.ros.org/kinetic/Installation). To create a workspace for Baxter, follow the directions [here](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup).

## TODO
### Baxter environments
- [ ] BaxterReacherEnv (in progress)
- [ ] BaxterPusherEnv (in progress)
- [ ] BaxterSliderEnv (in progress)
- [ ] BaxterPickPlaceEnv (in progress)
