#!/usr/bin/python3

import numpy as np

#--------------------------------------------------------------------------------
class Vehicle():
  def __init__(self,ID,source_strength = 0, imag_source_strength = 0.4):
    self.ID              = ID
    self.source_strength = source_strength
    self.imag_source_strength = imag_source_strength

    self.position        = np.zeros(3)
    self.velocity        = np.zeros(3)

    self.altitude        = 0
    self.goal            = np.zeros(3)
    self.sink_strength   = 0
    self.V_inf           = np.zeros(3)
    self.safety          = 0


  def Set_Velocity(self,vel):
    self.velocity = vel

  def Set_Position(self,pos):
    self.position = np.array(pos)

  def Set_Goal(self,goal,goal_strength,safety):
    self.goal          = goal
    self.sink_strength = goal_strength
    self.safety = safety

  def Go_to_Goal(self,altitude,AoA,t_start,Vinf):
    self.altitude = altitude                                       # Cruise altitude
    self.V_inf    = np.array([Vinf*np.cos(AoA), Vinf*np.sin(AoA)]) # Freestream velocity. AoA is measured from horizontal axis, cw (+)tive


