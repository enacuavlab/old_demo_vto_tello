#!/usr/bin/python3
import matplotlib.pyplot as plt
import csv
import math
import copy
import pyclipper
import numpy as np
from itertools import compress

filename = "testFiltered.csv"

#--------------------------------------------------------------------------------
class Building():
  def __init__(self,vertices): # Buildings(obstacles) are defined by coordinates of their vertices.
    self.vertices = vertices
    
    panels = np.array([])
    rad = 0.2
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


#--------------------------------------------------------------------------------
def Flow_Velocity_Calculation(vehicles,buildings):
  print("Flow_Velocity_Calculation")
  for f,vehicle in enumerate(vehicles):
    othervehicleslist = vehicles[:f] + vehicles[f+1:]
    vehicle.altitude_mask = np.zeros(( len(buildings) )) #, dtype=int) 
    for index,panelledbuilding in enumerate(buildings):
      if (panelledbuilding.vertices[:,2] > vehicle.altitude).any():
        vehicle.altitude_mask[index] = 1
    related_buildings = list(compress(buildings,vehicle.altitude_mask))

    for building in related_buildings:
      building.gamma_calc(vehicle,othervehicleslist)

  flow_vels = np.zeros([len(vehicles),3])
  V_gamma   = np.zeros([len(vehicles),2]) # Velocity induced by vortices
  V_sink    = np.zeros([len(vehicles),2]) # Velocity induced by sink element
  V_source  = np.zeros([len(vehicles),2]) # Velocity induced by source elements
  V_sum     = np.zeros([len(vehicles),2]) # V_gamma + V_sink + V_source
  V_normal  = np.zeros([len(vehicles),2]) # Normalized velocity
  V_flow    = np.zeros([len(vehicles),2]) # Normalized velocity inversly proportional to magnitude
  V_norm    = np.zeros([len(vehicles),1]) # L2 norm of velocity vector
  W_sink    = np.zeros([len(vehicles),1]) # Velocity induced by 3-D sink element
  W_source  = np.zeros([len(vehicles),1]) # Velocity induced by 3-D source element
  W_flow    = np.zeros([len(vehicles),1]) # Vertical velocity component (to be used in 3-D scenarios)
  W_sum     = np.zeros([len(vehicles),1])
  W_norm    = np.zeros([len(vehicles),1])
  W_normal  = np.zeros([len(vehicles),1])

  for f,vehicle in enumerate(vehicles):
    othervehicleslist = vehicles[:f] + vehicles[f+1:]

    # Velocity induced by 2D point sink, eqn. 10.2 & 10.3 in Katz & Plotkin:
    V_sink[f,0] = (-vehicle.sink_strength*(vehicle.position[0]-vehicle.goal[0]))/(2*np.pi*((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2))
    V_sink[f,1] = (-vehicle.sink_strength*(vehicle.position[1]-vehicle.goal[1]))/(2*np.pi*((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2))
    # Velocity induced by 3-D point sink. Katz&Plotkin Eqn. 3.25
    W_sink[f,0] = (-vehicle.sink_strength*(vehicle.position[2]-vehicle.goal[2]))/(4*np.pi*(((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2+(vehicle.position[2]-vehicle.goal[2])**2)**1.5))

    # Velocity induced by 2D point source, eqn. 10.2 & 10.3 in Katz & Plotkin:
    for othervehicle in othervehicleslist:
      V_source[f,0] += (othervehicle.source_strength*(vehicle.position[0]-othervehicle.position[0]))/(2*np.pi*((vehicle.position[0]-othervehicle.position[0])**2+(vehicle.position[1]-othervehicle.position[1])**2))
      V_source[f,1] += (othervehicle.source_strength*(vehicle.position[1]-othervehicle.position[1]))/(2*np.pi*((vehicle.position[0]-othervehicle.position[0])**2+(vehicle.position[1]-othervehicle.position[1])**2))
      W_source[f,0] += (othervehicle.source_strength*(vehicle.position[2]-othervehicle.position[2]))/(4*np.pi*((vehicle.position[0]-othervehicle.position[0])**2+(vehicle.position[1]-othervehicle.position[1])**2+(vehicle.position[2]-othervehicle.position[2])**2)**(3/2))

    for building in buildings:
      u = np.zeros((building.nop,1))
      v = np.zeros((building.nop,1))
      if vehicle.ID in building.gammas.keys():
        global a, b, c, d, e # Velocity induced by vortices on each panel: 
        u = ( building.gammas[vehicle.ID][:].T/(2*np.pi))  *((vehicle.position[1]-building.pcp[:,1]) /((vehicle.position[0]-building.pcp[:,0])**2+(vehicle.position[1]-building.pcp[:,1])**2)) ####
        v = (-building.gammas[vehicle.ID][:].T/(2*np.pi))  *((vehicle.position[0]-building.pcp[:,0]) /((vehicle.position[0]-building.pcp[:,0])**2+(vehicle.position[1]-building.pcp[:,1])**2))
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

    W_sum[f] = W_sink[f] + W_source[f]
    if W_sum[f] != 0.:
      W_norm[f] = (W_sum[f]**2)**0.5
      W_normal[f] = W_sum[f] /W_norm[f]
      W_flow[f] = W_normal[f]/W_norm[f]
      W_flow[f] = np.clip(W_flow[f],-0.07, 0.07)
    else:
      W_flow[f] = W_sum[f]

    flow_vels[f,:] = [V_flow[f,0],V_flow[f,1],W_flow[f,0]]

  return flow_vels

#--------------------------------------------------------------------------------
def display(name,pts):
  plt.text(pts[0][0],0.7+pts[0][1],name,fontsize=12)
  for i,elt in enumerate(pts[:-1]): plt.plot([pts[i][0],pts[i+1][0]],[pts[i][1],pts[i+1][1]],color='red')
  plt.plot([pts[0][0],pts[len(pts)-1][0]],[pts[0][1],pts[len(pts)-1][1]],color='red')


def clocked(pts):
  angles = []
  (x0,y0)=(0,0)
  for i in pts: (x0,y0) = (i[0]+x0,i[1]+y0)
  (x0,y0) = (x0/len(pts),y0/len(pts))
  for i in pts: 
    (dx,dy) = (i[0]-x0,i[1]-y0)
    angles.append(math.degrees(math.atan2(dy,dx)))
  tmp = [copy.copy(x) for y,x in sorted(zip(angles,pts), key=lambda pair: pair[0])]
  for i,item in enumerate(tmp):  pts[i] = item


def init(buildingList):
  plt.xlim(-5, 5)
  plt.ylim(-5, 5)
  plt.grid()
  with open(filename, newline='') as csvfile:
    csvreader = csv.reader(csvfile)
    fields = next(csvreader)
    values = next(csvreader)
    buildings = {}
    for i in range(0,len(fields),3):
      buildingName,markerName  = fields[i].split(':')
      floatLst=[float(x)/1000.0 for x in values[i:i+3]]
      if (not buildings) or (not buildingName in buildings): buildings[buildingName]={markerName:floatLst}
      buildings[buildingName].update({markerName:floatLst})

    for item1 in buildings.items(): 
      pts = np.empty((len(item1[1].items()),2))
      for i,item2 in enumerate(item1[1].items()): pts[i] = np.array(item2[1][0:2])
      clocked(pts) 
      vertices = np.empty((len(item1[1].items()),3))
      for i,item2 in enumerate(item1[1].items()): vertices[i] = (pts[i][0],pts[i][1],4.2)
      print(item1[0])
      buildingList.append(Building(vertices))
      display(item1[0],pts)


#def init_test(buildingList):
#  buildings = [([[3.0, 2.0, 4.2], [2.75, 1.567, 4.2], [2.25, 1.567, 4.2], [2.0, 2.0, 4.2], [2.25, 2.433, 4.2], [2.75, 2.433, 4.2]]),
#               ([[1.0, 3.0, 4.5], [0.75, 2.567, 4.5], [0.25, 2.567, 4.5], [0.0, 3.0, 4.5], [0.25, 3.433, 4.5], [0.75, 3.433, 4.5]]), 
#               ([[1.0, 0.5, 4], [0.75, 0.067, 4], [0.25, 0.067, 4], [0.0, 0.5, 4], [0.25, 0.933, 4], [0.75, 0.933, 4]]), 
#               ([[-2.65, 1.5, 4.5], [-3.0, 1.15, 4.5], [-3.35, 1.5, 4.5], [-3.0, 1.85, 4.5]]),
#               ([[-2.65, -1.5, 4.5], [-3.0, -1.85, 4.5], [-3.35, -1.5, 4.5], [-3.0, -1.15, 4.5]]),
#               ([[-1.15, -0.2, 4.5], [-1.5, -0.55, 4.5], [-1.85, -0.2, 4.5], [-1.5, 0.15, 4.5]]), 
#               ([[1.5, -2.5, 4.2], [1, -2.5, 4.2], [1, -1.4, 4.2], [1.5, -1, 4.2]]),
#               ([[3.5, -2.5, 4.2], [3, -2.5, 4.2], [3, -1, 4.2], [3.5, -1.4, 4.2]])]
#  for b in buildings: buildingList.append(Building(np.array(b)))


def run(buildingList):
  acDict = {60:[('TELLO-ED4310')],65:[('TELLO-F0B594')]}
  sourceStrength = 0.95 # Tello repelance
  vehicleList = [];
  for ac in acDict: vehicleList.append(Vehicle(ac,sourceStrength))

  vehicleList[0].Go_to_Goal(1.4,0,0,0) # altitude,AoA,t_start,Vinf=0.5,0.5,1.5
  vehicleList[1].Go_to_Goal(1.4,0,0,0) # altitude,AoA,t_start,Vinf=0.5,0.5,1.5
  targetPos = np.array([2.0,-4.0,2.0])

  vehicleList[0].Set_Goal(targetPos,5,0.0)
  vehicleList[1].Set_Goal(targetPos,5,0.0)
  vehicleList[0].Set_Position([-4,4,0])
  vehicleList[1].Set_Position([-4,-4,0])
  vehicleList[0].Set_Velocity([0,0,0])
  vehicleList[1].Set_Velocity([0,0,0])

  plt.plot(vehicleList[0].position[0],vehicleList[0].position[1],color='red',marker='o',markersize=12)
  plt.plot(vehicleList[1].position[0],vehicleList[1].position[1],color='red',marker='o',markersize=12)
  plt.plot(targetPos[0],targetPos[1],color='green',marker='o',markersize=12)

  
  flow_vels = Flow_Velocity_Calculation(vehicleList,buildingList)
  print(flow_vels)


#--------------------------------------------------------------------------------
if __name__ == '__main__':
  buildingList = []
  init(buildingList)
  run(buildingList)
  plt.show()
