#!/usr/bin/python3

import numpy as np

import threading
import queue
import time


#------------------------------------------------------------------------------
class Thread_commandSim(threading.Thread):

  def __init__(self,quitflag,vehicles):
    threading.Thread.__init__(self)
    self.quitflag = quitflag
    self.vehicles = vehicles
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
          for elt in self.vehicles:

            if (elt == 888):  # simulated target will circle at constant speed
              if (self.vehicles[elt][0] == 1):
                theta = theta + target_speed * np.pi / 800.0
                step[0] = r*np.cos(theta)
                step[1] = r*np.sin(theta)
                step[2] = self.vehicles[elt][2](elt)[2]
                self.vehicles[elt][2].position = step
                targetpos = step
              else:
                targetpos = self.vehicles[elt][2](elt)
           
            else:
              if (self.vehicles[elt][0] == 1):
                pos = self.vehicles[elt][2](elt)
                deltapos = np.subtract(targetpos,pos)
                deltastep = deltapos * drone_speed / 250.0
                self.vehicles[elt][2].position = np.add(pos,deltastep)

    finally: 
      print("Thread_commandSim stop")


  def triggersim(self):
    if self.suspend: self.suspend = False
    else: self.suspend = True
    print(self.suspend)

