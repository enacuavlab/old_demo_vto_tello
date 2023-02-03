#!/usr/bin/python3

"""
sudo pip3 install pyqtgraph
"""
import time,psutil,sys
import numpy as np


from PyQt5 import QtCore,QtGui,QtWidgets
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
    self.w = pg.GraphicsLayoutWidget()
    self.layout = QtWidgets.QGridLayout()
    self.w.setLayout(self.layout)
    self.w.setWindowTitle('ENAC VTO')
    self.w.show()

    self.store_cpu = (0.0,0.0)
    self.avg_cpu = 0.0
    self.store_time = time.time()

    self.fps_text = QtWidgets.QLabel("TEXT")
    self.fps_text.setStyleSheet("QLabel{font-size: 40pt; color:rgba(226, 39, 134, 127)}")
    self.fps_text.sizeHint = lambda: pg.QtCore.QSize(100, 100)
    self.layout.addWidget(self.fps_text)


    glvw = gl.GLViewWidget()
    glvw.opts['distance'] = 20
    glvw.opts['elevation'] = 40
    glvw.opts['azimuth'] = 90

    self.plots = {}
    i = 0
    color = np.empty((self.vehicleNb,4))
    for elt in vehiclelstsim: 
      lab = gl.GLTextItem(text=str(elt.ID))
      glvw.addItem(lab)
      self.plots[elt.ID]=(self.get_listpos,lab)
      color[i] =  (1.0, 0.0, 0.0, 0.5)
      i = i+1
    for elt in rigidBodyDict: 
      lab = gl.GLTextItem(text=str(elt))
      glvw.addItem(lab)
      self.plots[elt]=(self.get_dicpos,lab)
      color[i] = (0.0, 1.0, 0.0, 0.5)
      i = i+1

    self.sp = gl.GLScatterPlotItem(size=0.5,color=color,pxMode=False)
    glvw.addItem(self.sp)
    glvw.addItem(gl.GLGridItem(size=QtGui.QVector3D(10,10,10)))
    self.layout.addWidget(glvw)
    glvw.sizeHint = lambda: pg.QtCore.QSize(100, 500)



  def update(self):

    curr_time = time.time()
    fps = 1/(curr_time - self.store_time)
    self.store_time = curr_time
  
    self.store_cpu = tuple(np.array(self.store_cpu)+ (1.0,psutil.cpu_percent()))
    if self.store_cpu[0] == 10:
      self.avg_cpu =  self.store_cpu[1]/10
      self.store_cpu = (0.0,0.0)
      self.fps_text.setText("FPS "+f'{fps:.2f}'+"             CPU "+f'{self.avg_cpu:.2f}')

    dummy = np.empty((self.vehicleNb, 3))
    for i,elt in enumerate(self.plots):
      dummy[i]=self.plots[elt][0](elt) # call register get function 
      self.plots[elt][1].setData(pos=dummy[i])

    self.sp.setData(pos=dummy)


  def start(self):
    timer = QtCore.QTimer()
    timer.timeout.connect(self.update)
    timer.start(10)
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
      QtWidgets.QApplication.instance().exec_()
