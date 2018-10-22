# Robotics Control Environments
`gym-robotics` provides an interface for performing reinforcement learning and control experiments on robots in simulation and on real hardware. `gym-robotics` uses PyBullet as the backend for simulation experiments and ROS as the backend for hardware experiments. `gym-robotics` also makes use of the [OpenAI Gym](https://gym.openai.com/) paradigm that is common for reinforcement learning environments.

## Dependencies
Python 2.7 \
PyBullet \
OpenAI Gym \
ROS Kinetic \
Baxter SDK \
PyKDL \
baxter_pykdl

## Install
To install gym-robotics:

.. code:: shell
git clone https://github.com/jmichaux/gym-robotics
cd gym-robotics
pip install -e .

.. code:: shell
To install pybullet:


## TODO
### Baxter environments
- [ ] BaxterReacherEnv (in progress)
- [ ] BaxterPusherEnv (in progress)
- [ ] BaxterSliderEnv (in progress)
- [ ] BaxterPickPlaceEnv (in progress)
