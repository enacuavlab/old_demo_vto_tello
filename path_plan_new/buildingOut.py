#!/usr/bin/python3

import numpy as np


#--------------------------------------------------------------------------------
#
# This class represent a building from it's coefficient matrix (panel method)
# Vortex strength is produced from this matrix
#
#--------------------------------------------------------------------------------
class BuildingOut():
  def __init__(self,name,vertices): # Buildings(obstacles) are defined by coordinates of their vertices.
    self.name = name
    self.vertices = np.array(vertices)
    self.pcp = None
    self.pb = None
    self.nop  = None           # Number of Panels
    self.gammas = {}           # Vortex Strenghts
    self.K_inv = None

  def gamma_calc(self,vehicle,othervehicles): # Calculates unknown vortex strengths by solving panel method eq.
    vel_sink   = np.zeros((self.nop,2))
    vel_source = np.zeros((self.nop,2))
    vel_source_imag = np.zeros((self.nop,2))
    RHS        = np.zeros((self.nop,1))
    vel_sink[:,0] = (-vehicle.sink_strength*(self.pcp[:,0]-vehicle.goal[0]))/(2*np.pi*((self.pcp[:,0]-vehicle.goal[0])**2+(self.pcp[:,1]-vehicle.goal[1])**2))
    vel_sink[:,1] = (-vehicle.sink_strength*(self.pcp[:,1]-vehicle.goal[1]))/(2*np.pi*((self.pcp[:,0]-vehicle.goal[0])**2+(self.pcp[:,1]-vehicle.goal[1])**2))
    vel_source_imag[:,0] = (vehicle.imag_source_strength*(self.pcp[:,0]-vehicle.position[0]))/(2*np.pi*((self.pcp[:,0]-vehicle.position[0])**2+(self.pcp[:,1]-vehicle.position[1])**2))
    vel_source_imag[:,1] = (vehicle.imag_source_strength*(self.pcp[:,1]-vehicle.position[1]))/(2*np.pi*((self.pcp[:,0]-vehicle.position[0])**2+(self.pcp[:,1]-vehicle.position[1])**2))

    for i,othervehicle in enumerate(othervehicles) :
      vel_source[:,0] += (othervehicle.source_strength*(self.pcp[:,0]-othervehicle.position[0]))/(2*np.pi*((self.pcp[:,0]-othervehicle.position[0])**2+(self.pcp[:,1]-othervehicle.position[1])**2))
      vel_source[:,1] += (othervehicle.source_strength*(self.pcp[:,1]-othervehicle.position[1]))/(2*np.pi*((self.pcp[:,0]-othervehicle.position[0])**2+(self.pcp[:,1]-othervehicle.position[1])**2))

    RHS[:,0]  = -vehicle.V_inf[0]  * np.cos(self.pb[:])  \
                  -vehicle.V_inf[1]  * np.sin(self.pb[:])  \
                  -vel_sink[:,0]     * np.cos(self.pb[:])  \
                  -vel_sink[:,1]     * np.sin(self.pb[:])  \
                  -vel_source[:,0]   * np.cos(self.pb[:])  \
                  -vel_source[:,1]   * np.sin(self.pb[:])  \
                  -vel_source_imag[:,0]  * np.cos(self.pb[:])  \
                  -vel_source_imag[:,1]  * np.sin(self.pb[:])  +vehicle.safety
    self.gammas[vehicle.ID] = np.matmul(self.K_inv,RHS)

