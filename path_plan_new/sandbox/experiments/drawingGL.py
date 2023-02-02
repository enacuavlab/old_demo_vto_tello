#!/usr/bin/python3

"""
sudo pip3 install pyqtgraph
"""
import time,psutil,sys
import numpy as np


from PyQt5 import QtCore,QtWidgets
import pyqtgraph.opengl as gl
import pyqtgraph as pg

#--------------------------------------------------------------------------------
class DrawingGL():

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
    self.vehicleNb = len(vehiclelstsim)+len(rigidBodyDict)

    self.app = QtWidgets.QApplication(sys.argv)
    self.w = gl.GLViewWidget()
    self.w.opts['distance'] = 30
    self.w.setWindowTitle('ENAC VTO')
#    self.w.setGeometry(0, 110, 1920, 1080)
    self.w.addItem(gl.GLGridItem())

    self.w.show()

    self.store_cpu = (0.0,0.0)
    self.avg_cpu = 0.0
    self.store_time = time.time()

    self.plots = {}
    i = 0
    color = np.empty((self.vehicleNb,4))
    for elt in vehiclelstsim: 
      self.plots[elt.ID]=(self.get_listpos)
      color[i] =  (1.0, 0.0, 0.0, 0.5)
      i = i+1
    for elt in rigidBodyDict: 
      self.plots[elt]=(self.get_dicpos)
      color[i] = (0.0, 1.0, 0.0, 0.5)
      i = i+1

    self.sp = gl.GLScatterPlotItem(size=0.5,color=color,pxMode=False)
    self.w.addItem(self.sp)


  def update(self):

    curr_time = time.time()
    fps = 1/(curr_time - self.store_time)
    self.store_time = curr_time
  
    self.store_cpu = tuple(np.array(self.store_cpu)+ (1.0,psutil.cpu_percent()))
    if self.store_cpu[0] == 10:
      self.avg_cpu =  self.store_cpu[1]/10
      self.store_cpu = (0.0,0.0)
    
    print("FPS "+f'{fps:.2f}'+"             CPU "+f'{self.avg_cpu:.2f}')

    dummy = np.empty((self.vehicleNb, 3))
    for i,elt in enumerate(self.plots):
      dummy[i]=self.plots[elt](elt) # call register get function 

    self.sp.setData(pos=dummy)


  def start(self):
    timer = QtCore.QTimer()
    timer.timeout.connect(self.update)
    timer.start(10)
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
      QtWidgets.QApplication.instance().exec_()
