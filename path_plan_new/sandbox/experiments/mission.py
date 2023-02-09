#!/usr/bin/python3

import threading
import queue
import time
import threading

#------------------------------------------------------------------------------
telloFreq = 10

#------------------------------------------------------------------------------
class Thread_mission(threading.Thread):

  def __init__(self,quitflag,commands,targetsim,rigidBodyDict,targetId):
    threading.Thread.__init__(self)
    self.quitflag = quitflag
    self.commands = commands
    self.targetsim = targetsim
    self.rigidBodyDict = rigidBodyDict
    self.targetId = targetId


  def run(self):
    self.commands.put(('command',))
    self.commands.put(('streamon',))
    time.sleep(1)
    self.commands.put(('takeoff',))
    time.sleep(7)
    self.guidanceLoop() # drone should be flying to have position from optitrack
    self.commands.put(('land',))


  def guidanceLoop(self):
    unvalidcpt = 0
    telloPeriod = 1/telloFreq
    try: 
      while not self.quitflag:
        time.sleep(telloPeriod)
        if not self.targetsim:

          heading = np.arctan2(targetPos[1]-v.position[1],targetPos[0]-v.position[0])

          if not self.rigidBodyDict[self.targetId].valid:
            unvalidcpt = unvalidcpt+1
            if unvalidcpt == 10: break
            else: continue
          else: unvalidcpt= 0
      #    position = self.rigidBodyDict[65].position

    finally: 
      print("Thread_mission stop")
