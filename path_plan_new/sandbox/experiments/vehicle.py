#!/usr/bin/python3

import numpy as np

#--------------------------------------------------------------------------------
class Vehicle():
  def __init__(self,ID):

    self.ID = ID
    self.source_strength = 0.95       # Tello repelance
    self.sink_strength = 5.0         # attraction force from goal
    self.imag_source_strength = 0.4  # repealance force from buildings
    self.position  = np.zeros(3)
    self.goal      = np.zeros(3)
    self.V_inf     = np.zeros(3) # Freestream velocity. AoA is measured from horizontal axis, cw (+)tive
