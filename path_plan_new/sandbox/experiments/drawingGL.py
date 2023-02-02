#!/usr/bin/python3

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

    self.app = QtWidgets.QApplication(sys.argv)
    self.w = gl.GLViewWidget()
    self.w.opts['distance'] = 40
    self.w.setWindowTitle('pyqtgraph example: GLLinePlotItem')
    self.w.setGeometry(0, 110, 1920, 1080)
    self.w.show()

    self.store_cpu = (0.0,0.0)
    self.avg_cpu = 0.0
    self.store_time = time.time()

    self.storepos = {}
    self.plots = {}
    for elt in vehiclelstsim: 
      self.storepos[elt.ID] = np.zeros(3)
      self.plots[elt.ID]=(self.get_listpos,gl.GLScatterPlotItem(color=([1.0, 1.0, 1.0, 0.5]),size=3))

    for elt in rigidBodyDict: 
      self.storepos[elt] = np.zeros(3)
      self.plots[elt]=(self.get_dicpos,gl.GLScatterPlotItem(color=([1.0, 1.0, 1.0, 0.5]),size=10))

  def update(self):

    curr_time = time.time()
    fps = 1/(curr_time - self.store_time)
    self.store_time = curr_time
  
    self.store_cpu = tuple(np.array(self.store_cpu)+ (1.0,psutil.cpu_percent()))
    if self.store_cpu[0] == 10:
      self.avg_cpu =  self.store_cpu[1]/10
      self.store_cpu = (0.0,0.0)
    
    print("FPS "+f'{fps:.2f}'+"             CPU "+f'{self.avg_cpu:.2f}')

    for elt in self.plots:
      curr=self.plots[elt][0](elt) # call register get function 
      prev=self.storepos[elt]

      pos = [[curr[0],prev[0]],[curr[1],prev[1]],[curr[2],prev[2]]]
     
     
      pos2 = [[9.95995996,9.18367347,-0.06932245][9.97997998,9.18367347,-0.05015246][10.,9.18367347,-0.0305197]]
      print(posi2)

      self.plots[elt][1].setData(pos=pos)


  def start(self):
    timer = QtCore.QTimer()
    timer.timeout.connect(self.update)
    timer.start(10)
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
      QtWidgets.QApplication.instance().exec_()
