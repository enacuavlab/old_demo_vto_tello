#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

vertices = np.array([
  [ 0.070078,-1.021371, 4.2 ],
  [-0.834514,-1.038646, 4.2 ],
  [-0.83802, -0.122632, 4.2 ],
  [ 0.058454,-0.113969, 4.2 ]])

#--------------------------------------------------------------------------------
class BuildingIn():
  def __init__(self,vertices): 

    panels = np.empty((0, 3))
    for index,vertice in enumerate(vertices):
      xyz1 = vertices[index]                            # Coordinates of the first vertice
      xyz2 = vertices[ (index+1) % vertices.shape[0] ]  # Coordinates of the next vertice
      panels = np.vstack((panels, (xyz1,xyz2)[1:]))
  
    nop = panels.shape[0]              # Number of Panels
  
    XYZ2 = panels                      # Coordinates of end point of panel
    XYZ1 = np.roll(panels,1,axis=0)    # Coordinates of the next end point of panel
  
    self.pcp = np.zeros((nop,2))            # Controlpoints: at 3/4 of panel
    self.pcp  = XYZ2 + (XYZ1-XYZ2)*0.75 
  
    vp  = np.zeros((nop,2))            # Vortex point: at 1/4 of panel
    vp  = XYZ2 + (XYZ1-XYZ2)*0.25
  
    self.pb  = np.zeros((nop,1))  # Panel Orientation; measured from horizontal axis, ccw (+)tive, in radians
    self.pb  = np.arctan2( ( XYZ2[:,1] - XYZ1[:,1] ) , ( XYZ2[:,0] - XYZ1[:,0] ) )  + np.pi/2
  
    K = np.zeros((nop,nop))
    for m in range(nop ):
      for n in range(nop ):
        K[m,n] = ( 1 / (2*np.pi)
                     * ( (self.pcp[m][1]-vp[n][1] ) * np.cos(self.pb[m] ) - ( self.pcp[m][0] - vp[n][0] ) * np.sin(self.pb[m] ) )
                     / ( (self.pcp[m][0]-vp[n][0] )**2 + (self.pcp[m][1] - vp[n][1] )**2 ) )
  
    self.K_inv = np.linalg.inv(K) # Inverse of coefficient matrix: (Needed for solution of panel method eqn.)
    self.vertices = vertices
  

#--------------------------------------------------------------------------------
class BuildingOut():
  def __init__(self,K_inv,pb,pcp):
    self.K_inv = K_inv
    self.pb = pb
    self.pcp = pcp
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
    self.sink_strength = 5.0
    self.imag_source_strength = 0.4
    self.position  = np.zeros(3)
    self.goal      = np.zeros(3)
    self.V_inf     = np.zeros(3) # Freestream velocity. AoA is measured from horizontal axis, cw (+)tive


#--------------------------------------------------------------------------------
#--------------------------------------------------------------------------------
def Flow_Velocity_Calculation(vehicles,buildings):

  for f,vehicle in enumerate(vehicles):
    for building in buildings:
      building.gamma_calc(vehicle)

#--------------------------------------------------------------------------------
def display_building(pts):

  for i,elt in enumerate(pts[:-1]):
    plt.plot([pts[i][0],pts[i+1][0]],[pts[i][1],pts[i+1][1]],color='blue')
  plt.plot([pts[0][0],pts[len(pts)-1][0]],[pts[0][1],pts[len(pts)-1][1]],color='blue')


#--------------------------------------------------------------------------------
def display_vehicle(targetPos,vehicleList,flow_vels):

  plt.plot(targetPos[0],targetPos[1],color='green',marker='o',markersize=12)
  for i,v in enumerate(vehicleList):
    plt.plot(vehicleList[i].position[0],vehicleList[i].position[1],color='red',marker='o',markersize=12)
    vspeed=(flow_vels[i]/np.linalg.norm(flow_vels[i]))
#    plt.arrow(vehicleList[i].position[0],vehicleList[i].position[1],vspeed[0],vspeed[1], fc="k", ec="k", \
#              head_width=0.05, head_length=0.1 )

#--------------------------------------------------------------------------------
if __name__ == '__main__':

  buildingListIn = []
  buildingListIn.append(BuildingIn(vertices))

  buildingListOut = []
  buildingListOut.append(BuildingOut(buildingListIn[0].K_inv,buildingListIn[0].pb,buildingListIn[0].pcp))

  vehicleList = []
  vehicleList.append(Vehicle(65))

  targetPos = np.array([4.0,0.0,2.0])

  vehicleList[0].position = [-3.0,4,0]
  vehicleList[0].goal = targetPos

  flow_vels = Flow_Velocity_Calculation(vehicleList,buildingListOut)

  plt.xlim(-5, 5)
  plt.ylim(-5, 5)
  plt.grid()

  display_building(buildingListIn[0].vertices)
  display_vehicle(targetPos,vehicleList,flow_vels)

  plt.show()

  print(gammas)
  
#  gammas[vehicle.ID] = gamma_calc(buildingListIn[0].K_inv,buildingListIn[0].pb)
