#!/usr/bin/python3

import numpy as np

telloSpeed = 1 # [0.1 .. 1.0]

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


  def update(self,position,velocity,heading,goal,strength):
    self.position = position
    self.velocity = velocity
    angle = heading - np.pi / 2
    self.heading = -np.arctan2(np.sin(angle), np.cos(angle))
    self.goal = goal
    self.sink_strength = strength


  def apply_flow(self,flow):

    norm = np.linalg.norm(flow)
    flow_vels = flow/norm
    limited_norm = np.clip(norm,0., telloSpeed)
    vel_enu = flow*limited_norm

    k = 100.
    def RBI(psi):
      cp = np.cos(psi)
      sp = np.sin(psi)
      return np.array([[cp, sp, 0.],
                       [-sp, cp, 0.],
                       [0., 0., 1.]])
    def norm_ang(x):
      while x > np.pi :
        x -= 2*np.pi
      while x < -np.pi :
        x += 2*np.pi
      return x

    heading = np.arctan2(self.goal[1]-self.position[1],self.goal[0]-self.position[0])
    heading = norm_ang(heading)
    V_err_enu = vel_enu - self.velocity
    R = RBI(self.heading)
    V_err_xyz = R.dot(V_err_enu)
    err_heading = norm_ang(norm_ang(heading) - self.heading)

    def clamp100(x: int) -> int:
      return max(-100, min(100, x))

#    cmd = 'rc {} {} {} {}'.format(
#      clamp100(int(-V_err_xyz[1]*k)), # left_right_velocity
#      clamp100(int(V_err_xyz[0]*k)),  # forward_backward_velocity
#      clamp100(int(V_err_xyz[2]*k)),  # up_down_velocity
#      clamp100(int(-err_heading*k)))  # yaw_velocity

    cmd_val = (
      clamp100(int(-V_err_xyz[1]*k)), 
      clamp100(int(V_err_xyz[0]*k)), 
      clamp100(int(V_err_xyz[2]*k)), 
      clamp100(int(-err_heading*k))) 

    return(cmd_val)
