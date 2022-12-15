#!/usr/bin/python3

import numpy as np
import pyclipper
import math

#--------------------------------------------------------------------------------
# 
# This class represent a building from it's clockwize non aligned vertices
# A coefficient matrix (panel method) is produced from the shape of this building
#
#--------------------------------------------------------------------------------
class BuildingIn():
  def __init__(self,name,vertices): # Buildings(obstacles) are defined by coordinates of their vertices.
    self.name = name
    self.vertices = vertices
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
