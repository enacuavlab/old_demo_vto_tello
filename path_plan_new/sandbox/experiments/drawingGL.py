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
class View3D():

  def __init__(self,nbvehicle,dist,elev,azim):
    self.glvw = gl.GLViewWidget()
    self.glvw.opts['distance'] = dist
    self.glvw.opts['elevation'] = elev
    self.glvw.opts['azimuth'] =azim
    self.color = np.empty((nbvehicle,4))
    self.nbcol = 0
    self.spd = gl.GLScatterPlotItem(size=0.5,color=self.color,pxMode=False)
    self.glvw.addItem(self.spd)
    self.glvw.addItem(gl.GLGridItem(size=QtGui.QVector3D(10,10,10)))
    self.txts = {}


  def addplot(self,ident,color):
    txt = gl.GLTextItem(text=str(ident))
    self.glvw.addItem(txt)
    self.txts[ident] = txt
    self.color[self.nbcol]=color
    self.nbcol = self.nbcol+1


  def update(self,dummy):
    self.spd.setData(pos=dummy)
    for i,elt in enumerate(self.txts):
      self.txts[elt].setData(pos=dummy[i])


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
    self.layout = QtWidgets.QGridLayout(self.w)
    self.w.setWindowTitle('ENAC VTO')
    self.w.setMinimumWidth(1200)
    self.w.show()

    self.store_cpu = (0.0,0.0)
    self.avg_cpu = 0.0
    self.store_time = time.time()

    self.fps_text = QtWidgets.QLabel("TEXT")
    self.fps_text.setStyleSheet("QLabel{font-size: 40pt; color:rgba(226, 39, 134, 127)}")

    self.layout.addWidget(self.fps_text,0,0,1,-1) # menus one on purpose !
    self.layout.setRowStretch(self.layout.rowCount(), 1)

#    self.layout.addWidget(self.fps_text,0,0,1,1)
#    self.startsim_btn = QtWidgets.QPushButton("StartSim")
#    self.startsim_btn.setFixedSize(QtCore.QSize(50, 50))
#    self.layout.addWidget(self.startsim_btn,0,1,1,-1) 
#    self.layout.setRowStretch(self.layout.rowCount(), 1)

    self.v1=View3D(self.vehicleNb,25,40,90)
    self.v2=View3D(self.vehicleNb,25,90,90)

    self.plots = {}
    i = 0
    color = np.empty((self.vehicleNb,4))
    for elt in vehiclelstsim: 
      self.plots[elt.ID]=(self.get_listpos)
      color[i] =  (1.0, 0.0, 0.0, 0.5)
      self.v1.addplot(elt.ID,color[i])
      self.v2.addplot(elt.ID,color[i])
      i = i+1
    for elt in rigidBodyDict: 
      self.plots[elt]=(self.get_dicpos)
      color[i] = (0.0, 1.0, 0.0, 0.5)
      self.v1.addplot(elt,color[i])
      self.v2.addplot(elt,color[i])
      i = i+1

    self.layout.addWidget(self.v1.glvw,1,0,1,1)
    self.layout.addWidget(self.v2.glvw,1,1,1,1)



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
      dummy[i]=self.plots[elt](elt) # call register get function 

    self.v1.update(dummy)
    self.v2.update(dummy)



  def start(self):
    timer = QtCore.QTimer()
    timer.timeout.connect(self.update)
    timer.start(10)
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
      QtWidgets.QApplication.instance().exec_()
