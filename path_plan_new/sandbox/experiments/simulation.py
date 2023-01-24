#!/usr/bin/python3

import threading
import queue
import time


#------------------------------------------------------------------------------
class Thread_commandSim(threading.Thread):

  def __init__(self,quitflag,drawing,simlst):
    threading.Thread.__init__(self)
    self.quitflag = quitflag
    self.simlst = simlst
    self.drawing = drawing

  def run(self):
    updatedlst = simlst
    try: 
      while not self.quitflag:
        time.sleep(1)
        for elt in updatedlst:
          elt.position = elt.position + vspeed * 0.1
        self.drawing.refresh(updatedlst)

    finally: 
      print("Thread_commandSim stop")
