#!/usr/bin/python3

import numpy as np

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
  
    pcp = np.zeros((nop,2))            # Controlpoints: at 3/4 of panel
    pcp  = XYZ2 + (XYZ1-XYZ2)*0.75 
  
    vp  = np.zeros((nop,2))            # Vortex point: at 1/4 of panel
    vp  = XYZ2 + (XYZ1-XYZ2)*0.25
  
    self.pb  = np.zeros((nop,1))  # Panel Orientation; measured from horizontal axis, ccw (+)tive, in radians
    self.pb  = np.arctan2( ( XYZ2[:,1] - XYZ1[:,1] ) , ( XYZ2[:,0] - XYZ1[:,0] ) )  + np.pi/2
  
    K = np.zeros((nop,nop))
    for m in range(nop ):
      for n in range(nop ):
        K[m,n] = ( 1 / (2*np.pi)
                     * ( (pcp[m][1]-vp[n][1] ) * np.cos(self.pb[m] ) - ( pcp[m][0] - vp[n][0] ) * np.sin(self.pb[m] ) )
                     / ( (pcp[m][0]-vp[n][0] )**2 + (pcp[m][1] - vp[n][1] )**2 ) )
  
    self.K_inv = np.linalg.inv(K) # Inverse of coefficient matrix: (Needed for solution of panel method eqn.)
  
    print(self.K_inv)
    print(self.pb)


#--------------------------------------------------------------------------------
class BuildingOut():
  def __init__(self,K_inv,pb):
    self.K_inv = K_inv
    self.pb = pb

#  def gamma_calc(self,vehicle,othervehicles):
#    RHS[:,0]  = -vehicle.V_inf[0]  * np.cos(self.pb[:])  \
#                  -vehicle.V_inf[1]  * np.sin(self.pb[:])  \
#                  -vel_sink[:,0]     * np.cos(self.pb[:])  \
#                  -vel_sink[:,1]     * np.sin(self.pb[:])  \
#                  -vel_source[:,0]   * np.cos(self.pb[:])  \
#                  -vel_source[:,1]   * np.sin(self.pb[:])  \
#                  -vel_source_imag[:,0]  * np.cos(self.pb[:])  \
#                  -vel_source_imag[:,1]  * np.sin(self.pb[:])  +vehicle.safety
#
#    self.gammas[vehicle.ID] = np.matmul(self.K_inv,RHS)



#--------------------------------------------------------------------------------
if __name__ == '__main__':

  buildingListIn = []
  buildingListIn.append(BuildingIn(vertices))

  buildingListOut = []
  buildingListOut.append(BuildingOut(buildingListIn[0].K_inv,buildingListIn[0].pb))

  print(buildingListOut[0].K_inv)
  print(buildingListOut[0].pb)
