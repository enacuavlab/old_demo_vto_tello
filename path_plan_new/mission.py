#!/usr/bin/python3

from common import Flow_Velocity_Calculation

from shapely.geometry import Point, Polygon

import threading

import numpy as np
import queue
import time

telloSpeed = 0.3
telloFreq = 10

#------------------------------------------------------------------------------
class Thread_mission(threading.Thread):

  def __init__(self,commands,targetId,rigidBodyDict,vehicles,arena):
    threading.Thread.__init__(self)
    self.commands = commands
    self.targetId = targetId
    self.rigidBodyDict = rigidBodyDict
    self.vehicles = vehicles
    self.arena = arena
    self.running = True
    for v in self.vehicles: v.Go_to_Goal(1.4,0,0,0) # altitude,AoA,t_start,Vinf=0.5,0.5,1.5

  def run(self):
    time.sleep(1)
    self.commands.put(('takeoff',))
    time.sleep(7)
    self.guidanceLoop(self.commands)
    self.commands.put(('land',))

  def stop(self):
    self.running = False

  def guidanceLoop(self,commands):
    unvalidcpt = 0

    telloPeriod = 1/telloFreq
    for i in range(1500):

      time.sleep(telloPeriod)

      if not self.rigidBodyDict[self.targetId].valid: 
        unvalidcpt = unvalidcpt+1
        if unvalidcpt == 10: break
        else: continue
      else: unvalidcpt= 0

      if all(not Point(self.rigidBodyDict[self.targetId].position[0],
                       self.rigidBodyDict[self.targetId].position[1]).within(Polygon(building.vertices)) 
             for building in self.arena.buildings):
        targetPos = self.rigidBodyDict[self.targetId].position

      for v in self.vehicles:
        v.Set_Goal(targetPos,5,0.0)
        v.update(self.rigidBodyDict[v.ID].position,self.rigidBodyDict[v.ID].velocity,self.rigidBodyDict[v.ID].heading)
  
      flow_vels = Flow_Velocity_Calculation(self.vehicles,self.arena.buildings)
      for i,v in enumerate(self.vehicles):
        norm = np.linalg.norm(flow_vels[i])
        flow_vels[i] = flow_vels[i]/norm
        limited_norm = np.clip(norm,0., 0.8)
        fixed_speed = telloSpeed
        vel_enu = flow_vels[i]*limited_norm
        heading = np.arctan2(targetPos[1]-v.position[1],targetPos[0]-v.position[0])
        v.Set_Desired_Velocity(vel_enu, method='None')
        commands.put((v.send_velocity_enu(v.velocity_desired, heading),v.ID))
