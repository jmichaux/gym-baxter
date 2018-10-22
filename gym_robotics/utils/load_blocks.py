import os,  inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0,parentdir)

import pybullet as p
import numpy as np
import copy
import math
import random
import pybullet_data
from pprint import pprint
import time
from pybullet_envs.bullet.kuka import Kuka

# cid = p.connect(p.UDP,"192.168.86.100")
cid = p.connect(p.SHARED_MEMORY)
if (cid<0):
  p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()

objects = [p.loadURDF("plane.urdf", 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,1.000000)]
# objects = [p.loadURDF("shape_sorter.urdf", 0.000000,0.000000,1.000000,0.000000,0.000000,0.000000,1.000000)]

# load blocks
blocks_path = "/home-nfs/jmichaux/code/envs/gym-baxter/gym_baxter/envs/assets/objects/blocks/"
obj_name = blocks_path + "block_0.urdf"
obj = [p.loadURDF(obj_name, 1.300000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]

# blocks_path = "/home-nfs/jmichaux/code/envs/gym-baxter/gym_baxter/envs/assets/objects/blocks/"
# obj_name = blocks_path + "block_1.urdf"
# obj = [p.loadURDF(obj_name, 1.400000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]

# blocks_path = "/home-nfs/jmichaux/code/envs/gym-baxter/gym_baxter/envs/assets/objects/blocks/"
# obj_name = blocks_path + "block_2.urdf"
# obj = [p.loadURDF(obj_name, 1.500000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]

# blocks_path = "/home-nfs/jmichaux/code/envs/gym-baxter/gym_baxter/envs/assets/objects/blocks/"
# obj_name = blocks_path + "block_3.urdf"
# obj = [p.loadURDF(obj_name, 1.600000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]

# blocks_path = "/home-nfs/jmichaux/code/envs/gym-baxter/gym_baxter/envs/assets/objects/blocks/"
# obj_name = blocks_path + "block_4.urdf"
# obj = [p.loadURDF(obj_name, 1.700000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]

# blocks_path = "/home-nfs/jmichaux/code/envs/gym-baxter/gym_baxter/envs/assets/objects/blocks/"
# obj_name = blocks_path + "block_6.urdf"
# obj = [p.loadURDF(obj_name, 1.800000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]

blocks_path = "/home-nfs/jmichaux/code/envs/gym-baxter/gym_baxter/envs/assets/objects/blocks/"
obj_name = blocks_path + "block_7.urdf"
obj = [p.loadURDF(obj_name, 1.200000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]

blocks_path = "/home-nfs/jmichaux/code/envs/gym-baxter/gym_baxter/envs/assets/objects/blocks/"
obj_name = blocks_path + "block_9.urdf"
obj = [p.loadURDF(obj_name, 1.000000,-0.700000,.750000,0.000000,0.707107,0.000000,0.707107)]

# assets_path = "/home-nfs/jmichaux/code/envs/gym-baxter/gym_baxter/envs/assets/objects/"
# obj_name = assets_path + "cube_small.urdf"
# obj = [p.loadURDF(obj_name, 1.300000, 0.00000,0.750000,0.000000,0.707107,0.000000,0.707107)]

# assets_path = "/home-nfs/jmichaux/code/envs/gym-baxter/gym_baxter/envs/assets/objects/"
# obj_name = assets_path + "planar_2dof.urdf"
# pos = [0.00000, 0.00000, 0.1000]
# orn = p.getQuaternionFromEuler([0, np.pi/2, 0])
# orn = [0.000000,0.000000,0.000000,1.000000]
# obj = [p.loadURDF(obj_name, 0.00000, 0.00000,5.50000,0.000000,0.000000,0.000000,1.000000), useFixedBase=True)]
# obj = [p.loadURDF(obj_name, basePosition=pos, baseOrientation=orn, useFixedBase=True)]
# objects = [p.loadURDF("jenga/jenga.urdf", 1.300000,-0.700000,0.750000,0.000000,0.707107,0.000000,0.707107)]

# block_objs = os.listdir(blocks_path)
# for block in block_objs:
#   if '.urdf' in block:
#     obj_name = blocks_path + block
#     z = 1 + random.randint(1,10)/10
#     obj = [p.loadURDF(obj_name, 0.000000,0.000000,0.4)]
# shift = [0,-0.02,0]
# meshScale=[0.1,0.1,0.1]
# for i in range(3):
#   filename = str(i) + ".obj"
#   block_path= "/home-nfs/jmichaux/code/envs/gym-baxter/gym_baxter/envs/assets/objects/blocks/" + filename
#   #the visual shape and collision shape can be re-used by all createMultiBody instances (instancing)
#   visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,fileName=block_path, rgbaColor=[1,1,1,1], specularColor=[0.4,.4,0], visualFramePosition=shift, meshScale=meshScale)
#   # collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=block_path, collisionFramePosition=shift,meshScale=meshScale)

p.setGravity(0,0,-10)

p.setRealTimeSimulation(1)
ref_time = time.time()

running_time = 360 # seconds
while True:
# while (time.time() < ref_time+running_time):
  p.setGravity(0,0,-10)
  p.stepSimulation()