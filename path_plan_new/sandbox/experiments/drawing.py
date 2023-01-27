#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


#--------------------------------------------------------------------------------
class Drawing():
  def __init__(self,vehiclelstsim,rigidBodyDict):

    self.vehiclelstsim= vehiclelstsim
    self.rigidBodyDict= rigidBodyDict
    self.fig, self.ax = plt.subplots()
    self.fig.subplots_adjust(left=0.25, bottom=0.25)
    self.ax.set_xlabel('Time [s]')
    self.ax = plt.axes(xlim=(-5,5),ylim=(-5,5))
    self.ax.grid()
    self.scat = self.ax.scatter([],[])


  def update(self,i):
    if self.vehiclelstsim : 
      data = np.array([[self.vehiclelstsim[0].position[0],self.vehiclelstsim[0].position[1],0,0]])
      for i,elt in enumerate(self.vehiclelstsim, start=1): data = np.append(data,np.array([[elt.position[0],elt.position[1],0,0]]),axis=0)
    if self.rigidBodyDict : 
      if not self.vehiclelstsim: 
        first = next(iter(self.rigidBodyDict))
        data = np.array([[self.rigidBodyDict[first].position[0],self.rigidBodyDict[first].position[1],0,0]])
        begin = 1
      else: begin = 0
      for k, v in list(self.rigidBodyDict.items())[begin:]: 
        elt = self.rigidBodyDict[k]
        data = np.append(data,np.array([[elt.position[0],elt.position[1],0,0]]),axis=0)

    self.scat.set_offsets(data[:, :2])                     # x and y
    self.scat.set_sizes(300 * abs(data[:, 2])**1.5 + 100)  # size
    self.scat.set_array(data[:, 3])                        # color
    return self.scat,


  def start(self):
    ani = animation.FuncAnimation(self.fig,self.update,interval=1,blit=True)
    plt.show()
