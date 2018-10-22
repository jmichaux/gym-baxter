import os
import random
import numpy as np 
import pybullet as p

class LoadYCB():
  '''
  Class for loading YCB object models in the pybullet simulator
  '''
  def __init__(self, num_items=10, ='', textured=False, seed=seed):
    self.base_path = os.path.expanduser("~") + 'code/envs/gym-baxter/data/ycb/'
    self.objects = os.listdir(self.base_path)
    self.num_samples = num_samples
    self.textured = textured
    self.meshScale = [1 , 1 , 1]
    self.shift = [0, -0.02, 0]
    self.xrange = 0
    self.yrange = 0

  def load_objects(self):
    obj_list = random.sample(self.objects, self.num_samples)
    for obj in obj_list:
      self._load_object(obj_name=obj)
    return
  
  def _load_object(self, obj_name, base_position=None):
    if base_position is None:
      # generate random position
      base_position = []   
    obj_path = objects_path + obj_name +'/google_16k/'
    collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=obj_path + "nontextured.stl", collisionFramePosition=shift, meshScale=meshScale)
    if textured:
      visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,fileName=obj_path + "textured.obj", rgbaColor=[1,1,1,1], specularColor=[0.4,.4,0], visualFramePosition=shift, meshScale=meshScale)
    else:
      visualShapeId = 0
    p.createMultiBody(baseMass=1, baseInertialFramePosition=[0,0,0], baseCollisionShapeIndex=collisionShapeId, baseVisualShapeIndex=visualShapeId, basePosition=base_position, useMaximalCoordinates=True)
    return
    

