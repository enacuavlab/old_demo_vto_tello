#!/usr/bin/python3

import numpy as np
import pyclipper
import math

import matplotlib.pyplot as plt

#--------------------------------------------------------------------------------
class Vehicle():
  def __init__(self,ID):

    self.ID = ID
    self.sink_strength = 5.0         # attraction force from goal
    self.imag_source_strength = 0.4  # repealance force from buildings
    self.position  = np.zeros(3)
    self.goal      = np.zeros(3)
    self.V_inf     = np.zeros(3) # Freestream velocity. AoA is measured from horizontal axis, cw (+)tive


#--------------------------------------------------------------------------------
#--------------------------------------------------------------------------------
def Flow_Velocity_Calculation(vehicles,buildings):

  V_sink    = np.zeros([len(vehicles),2]) # Velocity induced by sink element
  W_sink    = np.zeros([len(vehicles),1]) # Velocity induced by 3-D sink element
  V_gamma   = np.zeros([len(vehicles),2]) # Velocity induced by vortices
  V_source  = np.zeros([len(vehicles),2]) # Velocity induced by source elements
  V_sum     = np.zeros([len(vehicles),2]) # V_gamma + V_sink + V_source
  V_norm    = np.zeros([len(vehicles),1]) # L2 norm of velocity vector
  V_normal  = np.zeros([len(vehicles),2]) # Normalized velocity
  V_flow    = np.zeros([len(vehicles),2]) # Normalized velocity inversly proportional to magnitude
  W_flow    = np.zeros([len(vehicles),1]) # Vertical velocity component (to be used in 3-D scenarios)
  flow_vels = np.zeros([len(vehicles),3])


  for f,vehicle in enumerate(vehicles):
    # Velocity induced by 2D point sink, eqn. 10.2 & 10.3 in Katz & Plotkin:
    V_sink[f,0] = (-vehicle.sink_strength*(vehicle.position[0]-vehicle.goal[0]))/ \
                  (2*np.pi*((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2))
    V_sink[f,1] = (-vehicle.sink_strength*(vehicle.position[1]-vehicle.goal[1]))/ \
                  (2*np.pi*((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2))

    # Total velocity induced by all elements on map:
    V_sum[f,0] = V_gamma[f,0] + V_sink[f,0] + vehicle.V_inf[0] + V_source[f,0]
    V_sum[f,1] = V_gamma[f,1] + V_sink[f,1] + vehicle.V_inf[1] + V_source[f,1]

    # L2 norm of flow velocity:
    V_norm[f] = (V_sum[f,0]**2 + V_sum[f,1]**2)**0.5
    # Normalized flow velocity:
    V_normal[f,0] = V_sum[f,0]/V_norm[f]
    V_normal[f,1] = V_sum[f,1]/V_norm[f]

    # Flow velocity inversely proportional to velocity magnitude:
    V_flow[f,0] = V_normal[f,0]/V_norm[f]
    V_flow[f,1] = V_normal[f,1]/V_norm[f]

    flow_vels[f,:] = [V_flow[f,0],V_flow[f,1],W_flow[f,0]]

  return flow_vels

#--------------------------------------------------------------------------------
if __name__ == '__main__':

  vehicleList = []
  vehicleList.append(Vehicle(65))

  vehicleList[0].position = np.array([-4.0,4,2.0]) 
  vehicleList[0].goal = np.array([4.0,-4.0,2.0])

  tracks = {}
  for elt in vehicleList:
    tracks[elt.ID] = []
    tracks[elt.ID].append(elt.position)

  for timestep in range(1,12):
    flow_vels = Flow_Velocity_Calculation(vehicleList,[])
    for i,elt in enumerate(vehicleList):
      vspeed=(flow_vels[i]/np.linalg.norm(flow_vels[i]))
      elt.position = elt.position + vspeed
      tracks[elt.ID].append(elt.position)

  plt.xlim(-5, 5)
  plt.ylim(-5, 5)
  plt.grid()

  for elt in vehicleList:
    plt.plot(tracks[elt.ID][0][0],tracks[elt.ID][0][1],color='red',marker='o',markersize=12)
    plt.plot(elt.goal[0],elt.goal[1],color='green',marker='o',markersize=12)

  for tr1 in tracks:
    for i,tr2 in enumerate(tracks[tr1][:-1]):
      plt.plot([tracks[tr1][i][0],tracks[tr1][i+1][0]],[tracks[tr1][i][1],tracks[tr1][i+1][1]],color='blue')

  plt.show()

