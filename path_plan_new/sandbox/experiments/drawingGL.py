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
    self.colors = np.empty((nbvehicle,4))
    self.nbcol = 0
    self.spd = gl.GLScatterPlotItem(size=0.5,color=self.colors,pxMode=False)
    self.glvw.addItem(self.spd)
    self.glvw.addItem(gl.GLGridItem(size=QtGui.QVector3D(10,10,10)))
    self.txts = {}


  def addplot(self,ident,color):
    self.label = gl.GLTextItem(text=str(ident))
    self.glvw.addItem(self.label)
    self.txts[ident] = self.label
    self.colors[self.nbcol]=color
    self.nbcol = self.nbcol+1


  def update(self,newpos,newcolors):
    self.spd.setData(pos=newpos,color=newcolors)
    for i,elt in enumerate(self.txts):
      self.txts[elt].setData(pos=newpos[i])


#--------------------------------------------------------------------------------
class DrawingGL():


  def __init__(self,vehicles,triggersim,triggerreal):

    self.vehicles = vehicles
    self.triggersim = triggersim
    self.triggerreal = triggerreal
    self.vehicleNb = len(vehicles)

    self.app = QtWidgets.QApplication(sys.argv)
    self.w = pg.GraphicsLayoutWidget()

    self.w.setWindowTitle('ENAC VTO')
    self.w.setMinimumWidth(1200)
    self.w.show()

    self.store_cpu = (0.0,0.0)
    self.avg_cpu = 0.0
    self.store_time = time.time()

    self.lay1 = QtWidgets.QVBoxLayout(self.w)
    self.lay2 = QtWidgets.QHBoxLayout()
    self.lay3 = QtWidgets.QHBoxLayout()
    self.lay1.addLayout(self.lay2) 
    self.lay1.addLayout(self.lay3) 

    self.fps_text = QtWidgets.QLabel("")
    self.fps_text.setStyleSheet("QLabel{font-size: 40pt; color:rgba(226, 39, 134, 127)}")

    self.lay2.addWidget(self.fps_text)
    self.startsim_btn = QtWidgets.QPushButton("SIM")
    self.startreal_btn = QtWidgets.QPushButton("REAL")
    self.startsim_btn.setFixedSize(QtCore.QSize(100, 50))
    self.startreal_btn.setFixedSize(QtCore.QSize(100, 50))
    self.lay2.addWidget(self.startsim_btn)
    self.lay2.addWidget(self.startreal_btn)
    if (triggersim): self.startsim_btn.clicked.connect(self.triggersim.trigger)
    else: self.startsim_btn.setEnabled(False)
    if (triggerreal): self.startreal_btn.clicked.connect(self.triggerreal.trigger)
    else: self.startreal_btn.setEnabled(False)

    self.v1=View3D(self.vehicleNb,25,40,-90)
    self.v2=View3D(self.vehicleNb,25,90,-90)

    self.plots = {}
    i = 0
    self.colors = np.zeros((self.vehicleNb,4))
    self.positions = np.zeros((self.vehicleNb,3))

    for elt in vehicles:
      self.plots[elt]=vehicles[elt][1]
      if elt == 888: self.colors[i] =  (0.0, 1.0, 0.0, 50)
      else:
        if (vehicles[elt][0]): self.colors[i] =  (1.0, 0.0, 0.0, 50)
        else: self.colors[i] = (0.0, 0.0, 1.0, 50)
      self.v1.addplot(elt,self.colors[i])
      self.v2.addplot(elt,self.colors[i])
      i = i+1

    self.lay3.addWidget(self.v1.glvw)
    self.lay3.addWidget(self.v2.glvw)



  def update(self):

    curr_time = time.time()
    fps = 1/(curr_time - self.store_time)
    self.store_time = curr_time
  
    self.store_cpu = tuple(np.array(self.store_cpu)+ (1.0,psutil.cpu_percent()))
    if self.store_cpu[0] == 10:
      self.avg_cpu =  self.store_cpu[1]/10
      self.store_cpu = (0.0,0.0)
      self.fps_text.setText("FPS "+f'{fps:.2f}'+"             CPU "+f'{self.avg_cpu:.2f}')

    newcolors = np.copy(self.colors)   # to preserve original colors array
    for i,elt in enumerate(self.plots):
      if self.plots[elt].valid: self.positions[i] = self.plots[elt].position
      else: newcolors[i][3]=0

    self.v1.update(self.positions,newcolors)
    self.v2.update(self.positions,newcolors)


  def start(self):
    timer = QtCore.QTimer()
    timer.timeout.connect(self.update)
    timer.start(10)
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
      QtWidgets.QApplication.instance().exec_()
