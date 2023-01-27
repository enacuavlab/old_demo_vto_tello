#!/usr/bin/python3

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

import time
import signal

#------------------------------------------------------------------------------
class Vehicle():
  def __init__(self,ID):
    self.id = ID
    self.position = np.zeros(3)

#------------------------------------------------------------------------------
class Thread_mission(threading.Thread):

  def __init__(self,quitflag,vehiclelst):
    threading.Thread.__init__(self)
    self.quitflag = quitflag
    self.vehiclelst = vehiclelst

  def run(self):
    telloPeriod = 1/1
    speed = [0.2,0.3,0.0]
    global x,y
    try:
      while not self.quitflag:
        time.sleep(telloPeriod)
        for elt in self.vehiclelst:
          elt.position = np.add(elt.position,speed)

    finally:
      print("Thread_mission stop")

#------------------------------------------------------------------------------
class Flag(threading.Event):
  def __bool__(self):
    return self.is_set()


#------------------------------------------------------------------------------
def update(i,vehiclelst,scat):

  data = np.array([[vehiclelst[0].position[0],vehiclelst[0].position[1],0,0]])
  for i,elt in enumerate(vehiclelst, start=1): data = np.append(data,np.array([[elt.position[0],elt.position[1],0,0]]),axis=0)

  scat.set_offsets(data[:, :2])                     # x and y
  scat.set_sizes(300 * abs(data[:, 2])**1.5 + 100)  # size
  scat.set_array(data[:, 3])                        # color

  return scat,


#------------------------------------------------------------------------------
def main():

  fig, ax = plt.subplots()
  fig.subplots_adjust(left=0.25, bottom=0.25)
  ax.set_xlabel('Time [s]')
  ax = plt.axes(xlim=(-5,5),ylim=(-5,5))
  ax.grid()

  signal.signal(signal.SIGINT, signal.SIG_DFL)

  flag = Flag()

  vehiclelst = []
  vehicle1 = Vehicle(65)
  vehicle1.position = [-1.0,-1.0,0.0]
  vehiclelst.append(vehicle1)
  vehicle2 = Vehicle(66)
  vehicle2.position = [-1.0,-2.0,0.0]
  vehiclelst.append(vehicle2)

  threadMission = Thread_mission(flag,vehiclelst)
  threadMission.start()

  ani = animation.FuncAnimation(fig,update,fargs=(vehiclelst,ax.scatter([],[])),interval=1,blit=True)

  plt.show()

  threadMission.join()

#------------------------------------------------------------------------------
if __name__=="__main__":
  main()
