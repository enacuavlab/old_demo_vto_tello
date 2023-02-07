#!/usr/bin/python3

import numpy as np

import threading
import queue
import time


#------------------------------------------------------------------------------
class Thread_commandSim(threading.Thread):

  def __init__(self,quitflag,targsim,simlst,rigidBodyDict,targetId):
    threading.Thread.__init__(self)
    self.quitflag = quitflag
    self.targsim = targsim
    self.simlst = simlst
    self.rigidBodyDict = rigidBodyDict
    self.targetId = targetId

  def run(self):
    try: 
      step = np.zeros(3)
      #speed = [0.1,0.2,0.0]
      r = 0.05
      theta = 0
      while not self.quitflag:
        time.sleep(1/60)

        for i,elt in enumerate(self.simlst):
          if i==0:
            if elt.ID == 888:  # simulated target will circle at constant speed

              print(elt.position)
              theta = theta + np.pi/200
              step[0] = r*np.cos(theta)
              step[1] = r*np.sin(theta)

              elt.position = np.subtract(elt.position,step)
              targetpos = elt.position
            else:
              targetpos = self.rigidBodyDict[self.targetId].position # get real target 
          
          if (i > 0) or (i==0 and elt.ID != 888): 
            dx, dy = (targetpos[0] - elt.position[0], targetpos[1] - elt.position[1])
            stepx, stepy = (dx / 25., dy / 25.)
            elt.position = np.add(elt.position,step)
            print("HELLO")

    finally: 
      print("Thread_commandSim stop")
