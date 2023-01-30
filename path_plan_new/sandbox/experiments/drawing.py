#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


#--------------------------------------------------------------------------------
class Drawing():

  def get_listpos(self,arg):
    ret = [0,0]
    for i,elt in enumerate(self.vehiclelstsim):
      if arg == elt.ID:
        ret = ([self.vehiclelstsim[i].position[0],self.vehiclelstsim[i].position[1]])
    return(ret)
 

  def get_dicpos(self,arg):
    return([self.rigidBodyDict[arg].position[0],self.rigidBodyDict[arg].position[1]])


  def __init__(self,FPS,vehiclelstsim,rigidBodyDict):

    self.FPS = FPS
    self.vehiclelstsim= vehiclelstsim
    self.rigidBodyDict= rigidBodyDict
    self.fig, self.ax = plt.subplots()
    self.fig.subplots_adjust(left=0.25, bottom=0.25)
    self.ax.set_xlabel('Time [s]')
    self.ax = plt.axes(xlim=(-5,5),ylim=(-5,5))
    self.ax.grid()
    self.scat = self.ax.scatter([],[])

    self.artiststodraw = [self.scat]
    self.annotations = {}
    self.plots = {}
    for elt in vehiclelstsim: 
      self.annotations[elt.ID]=self.ax.annotate(elt.ID, xy=(0,0))
      self.artiststodraw.append(self.annotations[elt.ID])
      self.plots[elt.ID]=self.get_listpos
    for elt in rigidBodyDict: 
      self.annotations[elt]=self.ax.annotate(elt, xy=(0,0))
      self.artiststodraw.append(self.annotations[elt])
      self.plots[elt]=self.get_dicpos


  def update(self,i):

    data = np.array([[0,0,0,0]])

    for elt in self.plots:
      position = self.plots[elt](elt) # call register get function 
      data = np.append(data,np.array([[position[0],position[1],0,0]]),axis=0)
      self.annotations[elt].set_position((position[0],position[1]))

    self.scat.set_offsets(data[:, :2])                     # x and y
    self.scat.set_offsets(data[1:, :2])                     # x and y
    self.scat.set_sizes(300 * abs(data[1:, 2])**1.5 + 100)  # size
    self.scat.set_array(data[:, 3])                        # color

    return (self.artiststodraw)


  def start(self):
    ani = animation.FuncAnimation(self.fig,self.update,interval=1000/self.FPS,blit=True)
    plt.show()
