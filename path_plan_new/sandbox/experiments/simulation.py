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
    self.suspend = True

  def run(self):
    try: 
      drone_speed = 1
      target_speed = 1
      step = np.zeros(3)
      r = 4.0
      theta = 0
      while not self.quitflag:
        time.sleep(1/60)
        if not self.suspend:
          for i,elt in enumerate(self.simlst):
            if i==0:
              if elt.ID == 888:  # simulated target will circle at constant speed
  
                theta = theta + target_speed * np.pi / 800.0
                step[0] = r*np.cos(theta)
                step[1] = r*np.sin(theta)
                step[2] = elt.position[2]
                elt.position = step
                targetpos = elt.position
              else:
                targetpos = self.rigidBodyDict[self.targetId].position # get real target 
            
            if (i > 0) or (i==0 and elt.ID != 888): 
              deltapos = np.subtract(targetpos,elt.position)
              deltastep = deltapos * drone_speed / 250.0
              elt.position = np.add(elt.position,deltastep)

    finally: 
      print("Thread_commandSim stop")


  def triggersim(self):
    if self.suspend: self.suspend = False
    else: self.suspend = True
    print(self.suspend)

