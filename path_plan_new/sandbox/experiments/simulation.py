#!/usr/bin/python3

import numpy as np

import threading
import queue
import time


#------------------------------------------------------------------------------
class Thread_commandSim(threading.Thread):

  def __init__(self,quitflag,simlst):
    threading.Thread.__init__(self)
    self.quitflag = quitflag
    self.simlst = simlst

  def run(self):
    try: 
      #vspeed = np.zeros(3)
      speed = [0.1,0.2,0.0]
      r = 0.05
      theta = 0
      while not self.quitflag:
        time.sleep(1/60)
        for elt in self.simlst:
          theta = theta + np.pi/200
          speed[0] = r*np.cos(theta)
          speed[1] = r*np.sin(theta)

#          print(theta,speed)
          elt.position = np.add(elt.position,speed)

    finally: 
      print("Thread_commandSim stop")
