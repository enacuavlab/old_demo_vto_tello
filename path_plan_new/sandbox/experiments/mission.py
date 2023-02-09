#!/usr/bin/python3

import threading
import queue
import time
import threading

#------------------------------------------------------------------------------
telloFreq = 10

#------------------------------------------------------------------------------
class Thread_mission(threading.Thread):

  def __init__(self,quitflag,commands,vehicles):
    threading.Thread.__init__(self)
    self.quitflag = quitflag
    self.commands = commands
    self.vehicles = vehicles

    print(self.vehicles[888])

  def run(self):
#    self.commands.put(('command',))
#    self.commands.put(('streamon',))
#    time.sleep(1)
#    self.commands.put(('takeoff',))
#    time.sleep(7)
    self.guidanceLoop() # drone should be flying to have position from optitrack
#    self.commands.put(('land',))


  def guidanceLoop(self):
    unvalidcpt = 0
    telloPeriod = 1/telloFreq
    delay = 0
    try: 
      while not self.quitflag:
        delay = delay + 1
        time.sleep(telloPeriod)

#        if (self.vehicles[888][0]):   # real target
#            if not self.vehicles[888][2](888):x
#            .valid:
#
#          unvalidcpt = unvalidcpt+1
#          if unvalidcpt == 10: break
#          else: continue
#        else: unvalidcpt= 0
#        print(delay)
#        for elt in selt.vehicles:
#
#          heading = np.arctan2(targetPos[1]-v.position[1],targetPos[0]-v.position[0])


    finally: 
      print("Thread_mission stop")
