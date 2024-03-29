#!/usr/bin/python3

import numpy as np
import pyclipper
import math

import matplotlib.pyplot as plt

building1_name = "Building881"
building1_vertices = np.array([
  [ 0.070078,-1.021371, 4.2 ],
  [-0.834514,-1.038646, 4.2 ],
  [-0.83802, -0.122632, 4.2 ],
  [ 0.058454,-0.113969, 4.2 ]])

building2_name = "Building882"
building2_vertices = np.array([
  [ 2.070078,-1.021371, 4.2 ],
  [ 2.834514,-1.038646, 4.2 ],
  [ 2.83802, -0.122632, 4.2 ],
  [ 2.058454,-0.113969, 4.2 ]])

#--------------------------------------------------------------------------------
class BuildingIn():
  def __init__(self,name,vertices): # Buildings(obstacles) are defined by coordinates of their vertices.
    self.name = name
    self.vertices = vertices
    self.unflated = vertices
    panels = np.array([])

#    rad = 0.2
    rad = 0.15

    safetyfac = 1.1
    rad = rad * safetyfac
    scale = 1e6  # Inflate
    pco = pyclipper.PyclipperOffset()
    pco.AddPath( (self.vertices[:,:2] * scale).astype(int).tolist() , pyclipper.JT_MITER, pyclipper.ET_CLOSEDPOLYGON)
    inflated  =  np.array ( pco.Execute( rad*scale )[0] ) / scale
    height = self.vertices[0,2]
    points = np.hstack(( inflated , np.ones((inflated.shape[0],1)) *height ))
    Xavg = np.mean(points[:,0:1])
    Yavg = np.mean(points[:,1:2])
    angles = np.arctan2( ( Yavg*np.ones(len(points[:,1])) - points[:,1] ) , ( Xavg*np.ones(len(points[:,0])) - points[:,0] ) )
    sorted_angles = sorted(zip(angles, points), reverse = True)
    points_sorted = np.vstack([x for y, x in sorted_angles])

    self.vertices = points_sorted
    self.gammas = {}           # Vortex Strenghts
    self.K_inv = None

    size=0.01 # Panelize
    for index,vertice in enumerate(self.vertices): # Divides obstacle edges into smaller line segments, called panels.
      xyz1 = self.vertices[index]                                 # Coordinates of the first vertice
      xyz2 = self.vertices[ (index+1) % self.vertices.shape[0] ]  # Coordinates of the next vertice
      s    = ( (xyz1[0]-xyz2[0])**2 +(xyz1[1]-xyz2[1])**2)**0.5   # Edge Length
      n    = math.ceil(s/size)                                    # Number of panels given desired panel size, rounded up
      if n == 1:
        raise ValueError('Size too large. Please give a smaller size value.')
      if panels.size == 0:
        panels = np.linspace(xyz1,xyz2,n)[1:]
      else:
        panels = np.vstack((panels,np.linspace(xyz1,xyz2,n)[1:])) # Divide the edge into "n" equal segments:

    # Compute coeff matrix for Vortex method
    self.nop = panels.shape[0]    # Number of Panels
    self.pcp = np.zeros((self.nop,2))  # Controlpoints: at 3/4 of panel
    vp  = np.zeros((self.nop,2))  # Vortex point: at 1/4 of panel
    pl  = np.zeros((self.nop,1))  # Panel Length
    self.pb  = np.zeros((self.nop,1))  # Panel Orientation; measured from horizontal axis, ccw (+)tive, in radians
    XYZ2 = panels                      # Coordinates of end point of panel
    XYZ1 = np.roll(panels,1,axis=0)    # Coordinates of the next end point of panel
    self.pcp  = XYZ2 + (XYZ1-XYZ2)*0.75 # Controlpoints point at 3/4 of panel. #self.pcp  = 0.5*( XYZ1 + XYZ2 )[:,:2]
    vp   = XYZ2 + (XYZ1-XYZ2)*0.25 # Vortex point at 1/4 of panel.
    self.pb   = np.arctan2( ( XYZ2[:,1] - XYZ1[:,1] ) , ( XYZ2[:,0] - XYZ1[:,0] ) )  + np.pi/2
    K = np.zeros((self.nop,self.nop))
    for m in range(self.nop ):
      for n in range(self.nop ):
        K[m,n] = ( 1 / (2*np.pi)
                     * ( (self.pcp[m][1]-vp[n][1] ) * np.cos(self.pb[m] ) - ( self.pcp[m][0] - vp[n][0] ) * np.sin(self.pb[m] ) )
                     / ( (self.pcp[m][0]-vp[n][0] )**2 + (self.pcp[m][1] - vp[n][1] )**2 ) )
    self.K_inv = np.linalg.inv(K) # Inverse of coefficient matrix: (Needed for solution of panel method eqn.)


#--------------------------------------------------------------------------------
class BuildingOut():
  def __init__(self,K_inv,pb,pcp,vertices,unflated):
    self.K_inv = K_inv
    self.pb = pb
    self.pcp = pcp
    self.vertices  = vertices
    self.unflated  = unflated
    self.nop = pb.shape[0]
    self.gammas = {}       # Vortex Strenghts


  def gamma_calc(self,vehicle):

    RHS             = np.zeros((self.nop,1))
    vel_sink        = np.zeros((self.nop,2))
    vel_source      = np.zeros((self.nop,2))
    vel_source_imag = np.zeros((self.nop,2))

    vel_sink[:,0] = (-vehicle.sink_strength*(self.pcp[:,0]-vehicle.goal[0]))/ \
      (2*np.pi*((self.pcp[:,0]-vehicle.goal[0])**2+(self.pcp[:,1]-vehicle.goal[1])**2))

    vel_sink[:,1] = (-vehicle.sink_strength*(self.pcp[:,1]-vehicle.goal[1]))/ \
      (2*np.pi*((self.pcp[:,0]-vehicle.goal[0])**2+(self.pcp[:,1]-vehicle.goal[1])**2))

    vel_source_imag[:,0] = (vehicle.imag_source_strength*(self.pcp[:,0]-vehicle.position[0]))/ \
      (2*np.pi*((self.pcp[:,0]-vehicle.position[0])**2+(self.pcp[:,1]-vehicle.position[1])**2))

    vel_source_imag[:,1] = (vehicle.imag_source_strength*(self.pcp[:,1]-vehicle.position[1]))/ \
      (2*np.pi*((self.pcp[:,0]-vehicle.position[0])**2+(self.pcp[:,1]-vehicle.position[1])**2))

    RHS[:,0]  = -vehicle.V_inf[0]  * np.cos(self.pb[:])  \
                -vehicle.V_inf[1]  * np.sin(self.pb[:])  \
                -vel_sink[:,0]     * np.cos(self.pb[:])  \
                -vel_sink[:,1]     * np.sin(self.pb[:])  \
                -vel_source[:,0]   * np.cos(self.pb[:])  \
                -vel_source[:,1]   * np.sin(self.pb[:])  \
                -vel_source_imag[:,0]  * np.cos(self.pb[:])  \
                -vel_source_imag[:,1]  * np.sin(self.pb[:])

    self.gammas[vehicle.ID] = np.matmul(self.K_inv,RHS)

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
    for building in buildings:
      building.gamma_calc(vehicle)

  for f,vehicle in enumerate(vehicles):
    # Velocity induced by 2D point sink, eqn. 10.2 & 10.3 in Katz & Plotkin:
    V_sink[f,0] = (-vehicle.sink_strength*(vehicle.position[0]-vehicle.goal[0]))/ \
                  (2*np.pi*((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2))
    V_sink[f,1] = (-vehicle.sink_strength*(vehicle.position[1]-vehicle.goal[1]))/ \
                  (2*np.pi*((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2))

    for building in buildings:
      u = np.zeros((building.nop,1))
      v = np.zeros((building.nop,1))
      if vehicle.ID in building.gammas.keys():
        global a, b, c, d, e # Velocity induced by vortices on each panel:
        u = ( building.gammas[vehicle.ID][:].T/(2*np.pi))  *((vehicle.position[1]-building.pcp[:,1]) / \
                ((vehicle.position[0]-building.pcp[:,0])**2+(vehicle.position[1]-building.pcp[:,1])**2)) ####
        v = (-building.gammas[vehicle.ID][:].T/(2*np.pi))  *((vehicle.position[0]-building.pcp[:,0]) / \
                ((vehicle.position[0]-building.pcp[:,0])**2+(vehicle.position[1]-building.pcp[:,1])**2))
        V_gamma[f,0] = V_gamma[f,0] + np.sum(u)
        V_gamma[f,1] = V_gamma[f,1] + np.sum(v)
        a = building.gammas[vehicle.ID]
        b = building.pcp
        c = u
        d = v
        e = V_gamma

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

  buildingListIn = []
  buildingListIn.append(BuildingIn(building1_name,building1_vertices))
  buildingListIn.append(BuildingIn(building2_name,building2_vertices))

  buildingListOut = []
  for bld in buildingListIn:
    buildingListOut.append(BuildingOut(bld.K_inv,bld.pb,bld.pcp,bld.vertices,bld.unflated))

  tracks = {}
  for elt in vehicleList:
    tracks[elt.ID] = []
    tracks[elt.ID].append(elt.position)

  for timestep in range(1,12):
    flow_vels = Flow_Velocity_Calculation(vehicleList,buildingListOut)
    for i,elt in enumerate(vehicleList):
      vspeed=(flow_vels[i]/np.linalg.norm(flow_vels[i]))
      elt.position = elt.position + vspeed
      tracks[elt.ID].append(elt.position)

  plt.xlim(-5, 5)
  plt.ylim(-5, 5)
  plt.grid()

  for bld in buildingListOut:
    for i,elt in enumerate(bld.vertices[:-1]):
      plt.plot([bld.vertices[i][0],bld.vertices[i+1][0]],[bld.vertices[i][1],bld.vertices[i+1][1]],color='blue')
      plt.plot([bld.unflated[i][0],bld.unflated[i+1][0]],[bld.unflated[i][1],bld.unflated[i+1][1]],color='red')
    plt.plot([bld.vertices[0][0],bld.vertices[len(bld.vertices)-1][0]],[bld.vertices[0][1],bld.vertices[len(bld.vertices)-1][1]],color='blue')
    plt.plot([bld.unflated[0][0],bld.unflated[len(bld.unflated)-1][0]],[bld.unflated[0][1],bld.unflated[len(bld.unflated)-1][1]],color='red')


  for elt in vehicleList:
    plt.plot(tracks[elt.ID][0][0],tracks[elt.ID][0][1],color='red',marker='o',markersize=12)
    plt.plot(elt.goal[0],elt.goal[1],color='green',marker='o',markersize=12)

  for tr1 in tracks:
    for i,tr2 in enumerate(tracks[tr1][:-1]):
      plt.plot([tracks[tr1][i][0],tracks[tr1][i+1][0]],[tracks[tr1][i][1],tracks[tr1][i+1][1]],color='blue')

  plt.show()
