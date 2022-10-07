#!/usr/bin/python3
from NatNetClient import NatNetClient
from collections import deque

#from path_plan_w_panel import ArenaMap, Flow_Velocity_Calculation
#from path_plan_w_panel import Flow_Velocity_Calculation

import json
from itertools import compress
from building import Building
from vehicle import Vehicle

import numpy as np
import socket
import select
import threading
import time
import queue
import subprocess
import docker
#import pygame


acDict = {60:[('TELLO-ED4310')],65:[('TELLO-F0B594')]}
#acDict = {60:[('TELLO-ED4310')]}
#acDict = {65:[('TELLO-F0B594')]}
acTarg = [888,'Helmet']

optiFreq = 20
telloFreq = 10
telloSpeed = 0.3

sourceStrength = 0.95 # Tello repelance

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class ArenaMap():
  def __init__(self):
    self.panels = None
    self.wind = [0,0]
    self.windT = 0
    self.buildings = []

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class Rigidbody():
  def __init__(self,ac_id):
    self.ac_id = ac_id
    self.valid = False
    self.position = np.zeros(3)
    self.velocity = np.zeros(3)
    self.heading = 0.
    self.quat = np.zeros(4)


class Natnet2python():
  def __init__(self, rigidBodyDict, freq=optiFreq, server="127.0.0.1", dataport=int(1511), commandport=int(1510), vel_samples=int(4), verbose=False):
    self.freq = freq
    self.vel_samples = vel_samples
    self.rigidBodyDict = rigidBodyDict
    self.timestamp = dict([(rb, None) for rb in self.rigidBodyDict])
    self.period = 1. / self.freq
    self.track = dict([(rb, deque()) for rb in self.rigidBodyDict])
    self.natnet = NatNetClient(
      server=server,
      rigidBodyListListener=self.receiveRigidBodyList,
      dataPort=dataport,
      commandPort=commandport,
      verbose=verbose)

  def run(self):
    self.natnet.run()

  def stop(self):
    if self.natnet is not None:
      print("Shutting down NATNET ...")
      self.natnet.stop()
      self.natnet = None

  def store_track(self,ac_id, pos, t):
    self.track[ac_id].append((pos, t))
    if len(self.track[ac_id]) > self.vel_samples:
      self.track[ac_id].popleft()

  def receiveRigidBodyList(self,rigidBodyList, stamp ):
    for (ac_id, pos, quat, valid) in rigidBodyList:
      if ac_id not in self.rigidBodyDict: continue
      if not valid: 
        self.rigidBodyDict[ac_id].valid = False
        continue # skip if rigid body is not valid
      self.store_track(ac_id, pos, stamp)
      if self.timestamp[ac_id] is None or abs(stamp - self.timestamp[ac_id]) < self.period:
        if self.timestamp[ac_id] is None: self.timestamp[ac_id] = stamp; continue # too early for next message
      self.timestamp[ac_id] = stamp
      vel = [ 0., 0., 0. ]
      if len(self.track[ac_id]) >= self.vel_samples:
        nb = -1
        for (p2, t2) in self.track[ac_id]:
          nb = nb + 1
          if nb == 0:
            p1 = p2
            t1 = t2
          else:
            dt = t2 - t1
            if dt < 1e-5: continue
            vel[0] += (p2[0] - p1[0]) / dt
            vel[1] += (p2[1] - p1[1]) / dt
            vel[2] += (p2[2] - p1[2]) / dt
            p1 = p2
            t1 = t2
        if nb > 0:
          vel[0] /= nb
          vel[1] /= nb
          vel[2] /= nb
      dcm_0_0 = 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2])
      dcm_1_0 = 2.0 * (quat[0] * quat[1] - quat[3] * quat[2])
      self.rigidBodyDict[ac_id].quaternion=quat
      self.rigidBodyDict[ac_id].heading=np.arctan2(dcm_1_0, dcm_0_0)
      self.rigidBodyDict[ac_id].position=np.array([pos[0],pos[1],pos[2]])
      self.rigidBodyDict[ac_id].velocity=np.array([vel[0],vel[1],vel[2]])
      self.rigidBodyDict[ac_id].valid = True

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class Thread_batt(threading.Thread):
  def __init__(self,inoutDict):
    threading.Thread.__init__(self)
    self.inoutDict = inoutDict
    self.running = True

  def run(self):
    inoutList = [self.inoutDict[key][0] for key in self.inoutDict.keys()]
    servDict = {} 
    for key in self.inoutDict.keys(): servDict[self.inoutDict[key][2]]=key
  
    while self.running:
      try:
        ready_read, ready_write, exceptional = select.select(inoutList,[],[],None)
        for ready in ready_read:
          if self.running:
            data, server = ready.recvfrom(1024)
            batt=data.decode(encoding="utf-8")
            #print(str(servDict[server[1]])+" batt:"+str(batt))
      except socket.timeout:
        pass

  def stop(self):
    self.running = False

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class Thread_mission(threading.Thread):
  def __init__(self,commands,rigidBodyDict,vehicles,arena):
    threading.Thread.__init__(self)
    self.commands = commands
    self.rigidBodyDict = rigidBodyDict
    self.vehicles = vehicles
    self.arena = arena
    self.running = True
    for v in self.vehicles: v.Go_to_Goal(1.4,0,0,0) # altitude,AoA,t_start,Vinf=0.5,0.5,1.5

  def Flow_Velocity_Calculation(self,vehicles,buildings):
    print("Flow_Velocity_Calculation")
    for f,vehicle in enumerate(vehicles):
      othervehicleslist = vehicles[:f] + vehicles[f+1:]
      vehicle.altitude_mask = np.zeros(( len(buildings) )) #, dtype=int)
      for index,panelledbuilding in enumerate(buildings):
        if (panelledbuilding.vertices[:,2] > vehicle.altitude).any():
          vehicle.altitude_mask[index] = 1
      related_buildings = list(compress(buildings,vehicle.altitude_mask))
  
      for building in related_buildings:
        building.gamma_calc(vehicle,othervehicleslist,self.arena)
  
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



  def run(self):
    target_pos = np.zeros(3)
    
    for i in range(1000):
      if(not self.rigidBodyDict[acTarg[0]].valid): 
        if self.running:time.sleep(1)
      else: break
    if(self.rigidBodyDict[acTarg[0]].valid):
      print("start mission")
       
      for i in range(5): 
        if self.running:time.sleep(1)
      if self.running: self.commands.put(('takeoff',))
      for i in range(5):  
        if self.running:time.sleep(1)
  
      upLevel=0
      for v in self.vehicles:
        self.commands.put(("up "+str(upLevel),v.ID))
        upLevel=upLevel+50
  
      telloPeriod = 1/telloFreq
      for i in range(1000):
        if not self.rigidBodyDict[acTarg[0]].valid: continue
        if self.running:time.sleep(telloPeriod)
        targetPos = self.rigidBodyDict[acTarg[0]].position
        for v in self.vehicles: 
          v.Set_Goal(targetPos,5,0.0)
          v.update(self.rigidBodyDict[v.ID].position,self.rigidBodyDict[v.ID].velocity,self.rigidBodyDict[v.ID].heading)
        flow_vels = self.Flow_Velocity_Calculation(self.vehicles,self.arena.buildings)
        for i,v in enumerate(self.vehicles):
          norm = np.linalg.norm(flow_vels[i])
          flow_vels[i] = flow_vels[i]/norm
          limited_norm = np.clip(norm,0., 0.8)
          fixed_speed = telloSpeed
          vel_enu = flow_vels[i]*limited_norm
          heading = np.arctan2(targetPos[1]-v.position[1],targetPos[0]-v.position[0])
          v.Set_Desired_Velocity(vel_enu, method='None')
          self.commands.put((v.send_velocity_enu(v.velocity_desired, heading),v.ID))
  
      if self.running: self.commands.put(('land',))

    print("Thread mission stopped")


#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
def init(inoutDict):
  ret=True
  for key in acDict.keys(): 
    if acDict[key][0] in [elt.name for elt in  docker.DockerClient().containers.list()]:
      res = subprocess.run(['docker','exec',acDict[key][0],'/bin/ping','-c 1','-W 1','192.168.10.1'],capture_output=True,text=True)
      if ("100% packet loss" in res.stdout):break
      print(acDict[key][0]+" connected")
      res = subprocess.run(['docker','exec',acDict[key][0],'/usr/bin/env'],capture_output=True,text=True)
      tmp=res.stdout
      left="CMD_PORT="
      if (left in tmp):
        ip=(docker.DockerClient().containers.get(acDict[key][0]).attrs['NetworkSettings']['IPAddress'])
        port=int((tmp[tmp.index(left)+len(left):]).split()[0])
        inoutDict[key]=((ip,port))
  if inoutDict:
    for key in acDict.keys(): 
      if key in inoutDict: 
        if(len(inoutDict[key]))==1: ret=False
      else: ret=False

  else: ret=False
  return(inoutDict,ret)

#------------------------------------------------------------------------------
def main(inoutDict):
#  print("Matrix computing, wait 13sec ...")
#  arena = ArenaMap(version = 65)
#  arena.Inflate(radius = 0.2)
#  arena.Panelize(size=0.01)
#  arena.Calculate_Coef_Matrix()
#  print("Matrix Computed")

  retmat = {}
  with open("matrix.json", "r") as infile: retmat = json.load(infile)
  infile.close()

  arena = ArenaMap()
  for val1, val2, val3, val4, val5 in retmat.values(): 
    b = Building(np.array(val1))
    b.vertices = np.array(val1) # udpate vertices
    b.pcp = np.array(val2)
    b.pb = np.array(val3)
    b.nop = val4
    b.K_inv = np.array(val5)
    arena.buildings.append(b)


  vehicleList = [];
  rigidBodyDict = {};
  rigidBodyDict[acTarg[0]] = Rigidbody(acTarg[0])
  for ac in acDict:
    sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('172.17.0.1',inoutDict[ac][1]))
    inoutDict[ac]=((sock,inoutDict[ac][0],inoutDict[ac][1]))
    rigidBodyDict[ac]=Rigidbody(ac)
    vehicleList.append(Vehicle(ac,sourceStrength))

  commands = queue.Queue()

  threadBatt = Thread_batt(inoutDict)
  threadBatt.start()
  threadOpt = Natnet2python(rigidBodyDict,freq=20)
  threadOpt.run()
  threadMission = Thread_mission(commands,rigidBodyDict,vehicleList,arena)
  threadMission.start()

  commands.put(('command',))
  commands.put(('streamon',))
  commands.put(('downvision 0',))

  try:
    while True:
      while not commands.empty():
        vtupple=commands.get()
        if (len(vtupple)==2): 
          inoutDict[vtupple[1]][0].sendto(vtupple[0].encode(encoding="utf-8"),(inoutDict[vtupple[1]][1],inoutDict[vtupple[1]][2]))
        else:
          for ac in acDict: 
            inoutDict[ac][0].sendto(vtupple[0].encode(encoding="utf-8"),(inoutDict[ac][1],inoutDict[ac][2]))

      time.sleep(0.01)

  except KeyboardInterrupt:
    print("\nWe are interrupting the program\n")
    time.sleep(1)
    threadOpt.stop()
    threadBatt.stop()
    for ac in acDict: inoutDict[ac][0].close()
    print("mainloop stopped")


#------------------------------------------------------------------------------
if __name__=="__main__":
  inoutDict = {}
  inout, ret = init(inoutDict)
  ret=True
  if(ret):
    print("---------------------------------------")
    print("When Tello is started, it will connected within 14 seconds")
    print("if tello led do not blink pink, restart it !")
    print("---------------------------------------")
    main(inoutDict)
