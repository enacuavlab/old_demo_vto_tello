#!/usr/bin/python3

import numpy as np

import threading
import time


#------------------------------------------------------------------------------
class Thread_drawreal(threading.Thread):

  def __init__(self,quitflag,drawing,realdic):
    threading.Thread.__init__(self)
    self.quitflag = quitflag
    self.drawing = drawing
    self.realdic = realdic

  def run(self):
    try: 
      while not self.quitflag:
        time.sleep(.05)
        self.drawing.refreshdic(self.realdic)

    finally: 
      print("Thread_drawreal stop")
