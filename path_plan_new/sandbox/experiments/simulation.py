#!/usr/bin/python3

import numpy as np

import threading
import queue
import time


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

#------------------------------------------------------------------------------
class Thread_commandSim(threading.Thread):

  def __init__(self,quitflag,mobiles):
    threading.Thread.__init__(self)
    self.quitflag = quitflag
    self.mobiles = mobiles
    self.suspend = True


  def put(self,elt,vcmd):
    print(elt,vcmd)
    left_right_velocity = vcmd[0]
    forward_backward_velocit = vcmd[1]
    up_down_velocity = vcmd[2]
    yaw_velocity = vcmd[3]

    self.mobiles[elt].position


  def run(self):
    try: 
      drone_speed = 1
      target_speed = 1
      step = np.zeros(3)
      r = 4.0
      theta = 0
      print("runnins SIM")
      while not self.quitflag:
        time.sleep(1/60)
        if not self.suspend:
          for elt in self.mobiles:

            if (elt == 888):  # simulated target will circle at constant speed
              if not (self.mobiles[elt][0]):
#                theta = theta + target_speed * np.pi / 800.0
                step[0] = r*np.cos(theta)
                step[1] = r*np.sin(theta)
                pos = self.mobiles[elt][1].position
                step[2] = pos[2] 
                self.mobiles[elt][1].position = step
                targetpos = step
              else:
                targetpos = self.mobiles[elt][1].position
           
            else:            # simulated tellos speed control

              print(self.mobiles[elt][1].appliedspeed)


#              if not (self.vehicles[elt][0]):
#                (pos,valid) = self.vehicles[elt][2](elt) # call registered get pos function
#                deltapos = np.subtract(targetpos,pos)
#                deltastep = deltapos * drone_speed / 250.0
#                self.vehicles[elt][1].position = np.add(pos,deltastep)


    finally: 
      print("Thread_commandSim stop")


  def trigger(self):
    if self.suspend: self.suspend = False
    else: self.suspend = True
