#!/usr/bin/python3

import numpy as np

import threading
import queue
import time

#------------------------------------------------------------------------------
simuFreq = 20

#------------------------------------------------------------------------------
class Simbody():

  def __init__(self,ac_id):
    self.ac_id = ac_id
    self.valid = True
    self.position = np.zeros(3)
    self.velocity = np.zeros(3)
    self.heading = 0.
    self.quat = np.zeros(4)
    self.appliedspeed = np.zeros(4)  #left_right forward_backward up_down yaw


  def computemotion(self):

    if not np.all(((self.appliedspeed)==0)):
  
      deltapos = np.array([self.appliedspeed[0],self.appliedspeed[1],self.appliedspeed[2]])
  
      drone_speed = 0.2
  
      deltastep = deltapos * drone_speed / 250.0
      self.position = np.add(self.position,deltastep)
      self.appliedspeed = np.zeros(4)
 

#------------------------------------------------------------------------------
class Thread_commandSim(threading.Thread):

  def __init__(self,quitflag,mobiles):
    threading.Thread.__init__(self)
    self.quitflag = quitflag
    self.mobiles = mobiles
    self.suspend = True
    self.simuPeriod = 1/simuFreq



#  def put(self,elt,vcmd):
#    print(elt,vcmd)
#    roll = vcmd[0]
#    pitch = vcmd[1]
#    alt = vcmd[2]
#    yaw = vcmd[3]
#    self.mobiles[elt].position



  def run(self):
    try: 
      drone_speed = 1
      target_speed = 1
      step = np.zeros(3)
      r = 4.0
      theta = 0

      while not self.quitflag:
        starttime = time.time()

        if not self.suspend:

          for elt in self.mobiles:

            if (elt == 888):  # simulated target will circle at constant speed
              if not (self.mobiles[elt][0]):
                theta = theta + target_speed * np.pi / 800.0
                step[0] = r*np.cos(theta)
                step[1] = r*np.sin(theta)
                pos = self.mobiles[elt][1].position
                step[2] = pos[2] 
                self.mobiles[elt][1].position = step
                targetpos = step
              else:
                targetpos = self.mobiles[elt][1].position
           
            else:            # simulated tellos speed control

               if not (self.mobiles[elt][0]): self.mobiles[elt][1].computemotion()


#              if not (self.vehicles[elt][0]):
#                (pos,valid) = self.vehicles[elt][2](elt) # call registered get pos function
#                deltapos = np.subtract(targetpos,pos)
#                deltastep = deltapos * drone_speed / 250.0
#                self.vehicles[elt][1].position = np.add(pos,deltastep)

        # if not self.suspend:
        time.sleep(self.simuPeriod-(time.time()-starttime))


    finally: 
      print("Thread_commandSim stop")


  def trigger(self):
    if self.suspend: self.suspend = False
    else: self.suspend = True
