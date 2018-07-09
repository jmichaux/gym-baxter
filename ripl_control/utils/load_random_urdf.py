import os
import random
import numpy as np 
import pybullet as p

class LoadRandomURDF():
  '''
  Class for loading random URDFs in the pybullet simulator
  '''
  def __init__(self, num_items=10, ='', textured=False, seed=seed):
    self.base_path = os.path.expanduser("~") + 'code/envs/gym-baxter/data/random_urdfs/'
    self.objects = os.listdir(self.base_path)
    self.num_samples = num_samples
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
    urdf = objects_path + obj_name + '/' + obj_name + '.urdf'
    objects = [p.loadURDF(urdf, 0.000000, 0.000000,0.707107 ,0.000000, 0.000000, 0.000000, 1.000000)]
    return
    

