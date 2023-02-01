#!/usr/bin/python3

import time,psutil
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from matplotlib.text import Annotation
from mpl_toolkits.mplot3d import Axes3D

from mpl_toolkits.mplot3d import proj3d

#https://stackoverflow.com/questions/62463276/how-to-annotate-3d-plot-on-matplotlib

#--------------------------------------------------------------------------------
class Drawing3D():

  def get_listpos(self,arg):
    ret = np.zeros(3)
    for i,elt in enumerate(self.vehiclelstsim):
      if arg == elt.ID:
        ret = (self.vehiclelstsim[i].position)
    return(ret)
 

  def get_dicpos(self,arg):
    return (self.rigidBodyDict[arg].position)


  def __init__(self,FPS,vehiclelstsim,rigidBodyDict):

    self.FPS = FPS
    self.vehiclelstsim= vehiclelstsim
    self.rigidBodyDict= rigidBodyDict

    self.fig = plt.figure()
    self.ax = self.fig.add_subplot(projection='3d')
    self.ax.set_xlim(-5, 5)
    self.ax.set_ylim(-5, 5)
    self.ax.set_zlim(0, 10)

    self.store_cpu = (0.0,0.0)
    self.avg_cpu = 0.0
    self.store_time = time.time()
    self.fps_text = self.ax.set_title(label='')

    self.storepos = {}
    self.plots = {}
    for elt in vehiclelstsim: 
      self.storepos[elt.ID] = np.zeros(3)
      self.plots[elt.ID]=(self.get_listpos,\
                          self.ax.plot([], [], [], 'o',markersize=8)[0],\
                          self.ax.text2D(0,0,elt.ID,fontsize=22))

    for elt in rigidBodyDict: 
      self.storepos[elt] = np.zeros(3)
      self.plots[elt]=(self.get_dicpos,\
                       self.ax.plot([], [], [], 'o',markersize=8)[0],\
                       self.ax.text2D(0,0,elt,fontsize=22))

  def update(self,i):

    curr_time = time.time()
    fps = 1/(curr_time - self.store_time)
    self.store_time = curr_time
  
    self.store_cpu = tuple(np.array(self.store_cpu)+ (1.0,psutil.cpu_percent()))
    if self.store_cpu[0] == 10:
      self.avg_cpu =  self.store_cpu[1]/10
      self.store_cpu = (0.0,0.0)

    self.fps_text.set_text("FPS "+f'{fps:.2f}'+"             CPU "+f'{self.avg_cpu:.2f}')

    for elt in self.plots:
      curr=self.plots[elt][0](elt) # call register get function 
      prev=self.storepos[elt]
      self.plots[elt][1].set_data([curr[0],prev[0]],[curr[1],prev[1]])
      self.plots[elt][1].set_3d_properties([curr[2],prev[2]])
      self.storepos[elt] = curr
      x2, y2, _ = proj3d.proj_transform(curr[0],curr[1],curr[2],self.ax.get_proj())
      self.plots[elt][2].set_position((x2,y2))

    return (self.ax,) # coma because of blit=True


  def start(self):
#    ani = animation.FuncAnimation(self.fig,self.update,interval=1000/self.FPS,blit=True)
    ani = animation.FuncAnimation(self.fig,self.update,interval=1,blit=True)
    plt.show()
