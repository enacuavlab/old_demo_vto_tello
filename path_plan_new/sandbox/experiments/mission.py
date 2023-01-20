#!/usr/bin/python3

import threading
import queue
import time
import threading

#------------------------------------------------------------------------------
telloFreq = 10

#------------------------------------------------------------------------------
class Thread_mission(threading.Thread):

  def __init__(self,gs0,fig,quitflag,rigidBodyDict,targetId):
    threading.Thread.__init__(self)
    self.gs0 = gs0
    self.fig = fig
    self.quitflag = quitflag
    self.rigidBodyDict = rigidBodyDict
    self.targetId = targetId


  def run(self):
    self.guidanceLoop()

  def guidanceLoop(self):
    unvalidcpt = 0
    telloPeriod = 1/telloFreq
    try: 
      while not self.quitflag:
        time.sleep(telloPeriod)
        if not self.rigidBodyDict[self.targetId].valid:
          unvalidcpt = unvalidcpt+1
          if unvalidcpt == 10: break
          else: continue
        else: unvalidcpt= 0
        position = self.rigidBodyDict[65].position
        self.gs0.plot(position[0],position[1],color='green',marker='o',markersize=12)
        self.fig.canvas.draw_idle()

        print(position)

    finally: 
      print("Thread_mission stop")
