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
      while not self.quitflag:
        time.sleep(1)
        vspeed = np.zeros(3)
        for elt in self.simlst:
          elt.position = elt.position + vspeed * 0.1

    finally: 
      print("Thread_commandSim stop")