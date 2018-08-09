import time
import rospy
import pybullet as p
from baxter_ros import *
from baxter_pybullet import *

# initialize a node
rospy.init_node('ros_pybullet_test')
baxter_ros = BaxterRos("right")

# # connect to physics server
# cid = p.connect(p.SHARED_MEMORY)
# if (cid<0):
#     p.connect(p.GUI)
# # p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.resetSimulation()
# baxter_pybullet = BaxterPyBullet()
#
# p.setGravity(0.000000,0.000000,-10.000000)
# p.setRealTimeSimulation(1)
# ref_time = time.time()
#
# running_time = 360 # seconds
# time.sleep(10)

left = {12: 'left_s0',
         13: 'left_s1',
         14: 'left_e0',
         15: 'left_e1',
         16: 'left_w0',
         18: 'left_w1',
         19: 'left_w2',}

right = {34: 'right_s0',
         35: 'right_s1',
         36: 'right_e0',
         37: 'right_e1',
         38: 'right_w0',
         40: 'right_w1',
         41: 'right_w2',}



# ros_ee = 1000 * np.array(baxter_ros.left_arm.endpoint_pose()['position'])
# pybullet_ee = 1000 * np.array(p.getLinkState(baxter_pybullet.baxter_id, 48)[0])
# error = np.mean((ros_ee - pybullet_ee)**2)
# print("Position error: ", error, np.sqrt(error))

sim_joints = [[12, 0.0],
              [13, 0.0],
              [14, 0.0],
              [15, 0.0],
              [16, 0.0],
              [18, 0.0],
              [19, 0.0]]

real_joints = {'right_e0': -0.0011504855909140602,
               'right_e1': 0.00,
               'right_s0': -0.0007669903939427069,
               'right_s1': -0.00,
               'right_w0': -0.0019174759848567672,
               'right_w1': 0.00,
               'right_w2': -0.0019174759848567672}

start = time.time()
baxter_ros.right_arm.set_joint_positions(real_joints)
print("Time to move real robot: ", time.time() - start)

# time.sleep(5)

# start = time.time()
# baxter_pybullet.move_to(sim_joints)
# print("Time to move simulated robot: ", time.time() - start)
