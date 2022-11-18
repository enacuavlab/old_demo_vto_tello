import numpy as np
from numpy import linalg
import math
import matplotlib.pyplot as plt
import pyclipper
from shapely.geometry import Point, Polygon
from datetime import datetime
from itertools import compress
import time
import socket

import pdb

class Vehicle():
#  def __init__(self,ID,sock,source_strength = 0, imag_source_strength = 0.4):
  def __init__(self,ID,source_strength = 0, imag_source_strength = 0.4):
    self.position_enu = np.zeros(3)
    self.velocity_enu = np.zeros(3)
    self.heading = 0.
    self.last_rc_control_timestamp = time.time()
  
    self.altitude        = 0
    self.sink_strength   = 0
    self.V_inf           = np.zeros(3)
    self.safety          = 0

    self.t               = 0
    self.position        = np.zeros(3)
    self.velocity        = np.zeros(3)
    self.goal            = np.zeros(3)
    self.source_strength = source_strength
    self.imag_source_strength = imag_source_strength
    self.gamma           = 0
    self.altitude_mask   = None
    self.ID              = ID
    self.path            = []
    self.state           = 0
    self.distance_to_destination = None
    self.velocitygain    = 1/50 # 1/300 or less for vortex method, 1/50 for hybrid
    self.velocity_desired = np.zeros(3)
    self.velocity_corrected = np.zeros(3)
    self.vel_err = np.zeros(3)

  def Set_Position(self,pos):
    self.position = np.array(pos)
    self.path     = np.array(pos)
    # print('GOOOAAALLL : ', self.goal)
    if np.all(self.goal) != None:
      self.distance_to_destination = np.linalg.norm(np.array(self.goal)-np.array(self.position))
      if np.all(self.distance_to_destination) < 0.2:
        self.state = 1

  def Set_Velocity(self,vel):
    self.velocity = vel

  def Set_Desired_Velocity(self,vel, method='direct'):
    self.velocity_desired = vel
    self.correct_vel(method=method)


  def correct_vel(self, method='None'):
    if method == 'projection':
      wind = self.velocity - self.velocity_desired
      self.vel_err = self.vel_err - (wind - np.dot(wind, self.velocity_desired/np.linalg.norm(self.velocity_desired) ) * 
              np.linalg.norm(self.velocity_desired) ) *(1./240.)
    elif method == 'direct':
      # err = self.velocity_desired - self.velocity
      self.vel_err = (self.velocity_desired - self.velocity)*(1./40.)
      # self.vel_err = (self.velocity_desired - self.velocity)
      # print(f' Vel err : {self.vel_err[0]:.3f}  {self.vel_err[1]:.3f}  {self.vel_err[2]:.3f}')
    else:
      self.vel_err = np.zeros(3)
            
    self.velocity_corrected = self.velocity_desired + self.vel_err
    self.velocity_corrected[2] = 0.


  def Set_Goal(self,goal,goal_strength,safety):
    self.goal          = goal
    self.sink_strength = goal_strength
    self.safety = safety

  def Set_Next_Goal(self,goal, goal_strength=500):
    self.state         = 0
    self.goal          = goal
    # self.sink_strength = goal_strength NOT USED FOR NOW

  def Go_to_Goal(self,altitude,AoA,t_start,Vinf):
    self.altitude = altitude                                       # Cruise altitude
    self.V_inf    = np.array([Vinf*np.cos(AoA), Vinf*np.sin(AoA)]) # Freestream velocity. AoA is measured from horizontal axis, cw (+)tive
    self.t = t_start

  def update(self,position,velocity,heading):
    self.position_enu = position
    self.velocity_enu = velocity
    angle = heading - np.pi / 2
    self.heading = -np.arctan2(np.sin(angle), np.cos(angle))

    self.Set_Position(position)
    self.Set_Velocity(velocity)

  #-----------------------------------------------------------------------------
  def send_rc_control(self, left_right_velocity: int, forward_backward_velocity: int, up_down_velocity: int, yaw_velocity: int):
    def clamp100(x: int) -> int:
      return max(-100, min(100, x))

    if time.time() - self.last_rc_control_timestamp > 0.001:
      self.last_rc_control_timestamp = time.time()
      cmd = 'rc {} {} {} {}'.format(
        clamp100(left_right_velocity),
        clamp100(forward_backward_velocity),
        clamp100(up_down_velocity),
        clamp100(yaw_velocity)
      )
      return(cmd)

  #-----------------------------------------------------------------------------
  def send_velocity_enu(self, vel_enu, heading):
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

    heading = norm_ang(heading)
    V_err_enu = vel_enu - self.velocity_enu
    R = RBI(self.heading)
    V_err_xyz = R.dot(V_err_enu)
    err_heading = norm_ang(norm_ang(heading) - self.heading)
    return(self.send_rc_control(int(-V_err_xyz[1]*k),int(V_err_xyz[0]*k),int(V_err_xyz[2]*k), int(-err_heading*k)))

  #-----------------------------------------------------------------------------
  def fly_to_enu(self,position_enu, heading=None):
    if heading is None:
      heading = self.heading
    pos_error = position_enu - self.position_enu
    vel_enu = pos_error*1.2 - self.velocity_enu
    return(self.send_velocity_enu(vel_enu, heading))
