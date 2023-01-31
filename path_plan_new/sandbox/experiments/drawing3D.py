#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from matplotlib.text import Annotation
from mpl_toolkits.mplot3d import Axes3D

from mpl_toolkits import mplot3d as plt3d

from typing import Tuple,Union

https://stackoverflow.com/questions/62463276/how-to-annotate-3d-plot-on-matplotlib

#--------------------------------------------------------------------------------
class Annotation3D(Annotation):

  def __init__(self,text:str,xyz: Tuple[float, float, float],*args,**kwargs):
    print("heelo")    
    Annotation.__init__(self, text, xy=(0, 0), *args, **kwargs)
    self._xyz = xyz
    self._xy: Optional[Tuple[float, float]] = None

  @property
  def xy(self):
    if self._xy is not None:
      return self._xy
    *xy2d, _ = plt3d.proj3d.proj_transform(*self._xyz, self.axes.M)
    return xy2d

  @xy.setter
  def xy(self, val):
    self._xy = val

  @xy.deleter
  def xy(self):
    self._xy = None

#--------------------------------------------------------------------------------
def annotateND(ax: Union[plt.Axes, plt3d.axes3d.Axes3D], text: str, \
               xy: Tuple[float, ...], *args, **kwargs) -> Union[Annotation3D, Annotation]:
  if isinstance(ax, plt3d.axes3d.Axes3D):
    a = Annotation3D(text, xy, *args, **kwargs)
    ax.add_artist(a)
    return a
  return ax.annotate(text, xy, *args, **kwargs)

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


    self.annotations = {}
    self.storepos = {}
    self.plots = {}
    self.artiststodraw = []
    for elt in vehiclelstsim: 
      self.annotations[elt.ID]=annotateND(self.ax,"HELLO",xy=(0,0))
#      self.annotations[elt.ID]=self.ax.annotateND(elt.ID, xy=(0,0),va='center',ha='center')
#      self.artiststodraw.append(self.annotations[elt.ID])
      scat = self.ax.plot([], [], [], 'o',markersize=8)[0]
      self.storepos[elt.ID] = np.zeros(3)
      self.plots[elt.ID]=(self.get_listpos,scat)
      self.artiststodraw.append(scat)

    for elt in rigidBodyDict: 
#      self.annotations[elt]=self.ax.annotate(elt, xy=(0,0),va='center',ha='center')
#      self.artiststodraw.append(self.annotations[elt])
      scat = self.ax.plot([], [], [], 'o',markersize=8)[0]
      self.storepos[elt] = np.zeros(3)
      self.plots[elt]=(self.get_dicpos,scat)
      self.artiststodraw.append(scat)


  def update(self,i):

    for elt in self.plots:
      curr=self.plots[elt][0](elt) # call register get function 
      prev=self.storepos[elt]
      self.plots[elt][1].set_data([curr[0],prev[0]],[curr[1],prev[1]])
      self.plots[elt][1].set_3d_properties([curr[2],prev[2]])
      self.storepos[elt] = curr
#      self.annotations[elt].set_position((curr[0],curr[1],curr[2]))

      
    return (self.artiststodraw)


  def start(self):
    ani = animation.FuncAnimation(self.fig,self.update,interval=1000/self.FPS,blit=True)
    plt.show()
