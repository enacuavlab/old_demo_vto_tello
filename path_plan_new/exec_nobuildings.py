#!/usr/bin/python3

from natnet import Thread_natnet

import threading
import numpy as np
import subprocess
import queue
import socket
import time

#------------------------------------------------------------------------------
tellos_routeur = {61:'TELLO-ED433E',62:'TELLO-ED4317',63:'TELLO-ED42A3',64:'TELLO-ED4381',65:'TELLO-F0B594',66:'TELLO-99CE21'}
tellos_docker = {60:'TELLO-ED4310',67:'TELLO-99CE5A',68:'TELLO-99CE4E'}

#tellos_selected = (65,)
tellos_selected = (65,66)

acTarg = [888,'Helmet']


telloFreq = 10

telloSpeed = 1 # [0.1 .. 1.0] 

optiFreq = 20 # Check that optitrack stream at least with this value


#------------------------------------------------------------------------------
class Rigidbody(): # should be compliant with Thread_natnet 
                   # TODO put in natnet.py

  def __init__(self,ac_id):
    self.ac_id = ac_id
    self.valid = False
    self.position = np.zeros(3)
    self.velocity = np.zeros(3)
    self.heading = 0.
    self.quat = np.zeros(4)

#------------------------------------------------------------------------------
class Vehicle():

  def __init__(self,ID):
    self.ID = ID
    self.position = np.zeros(3)
    self.velocity = np.zeros(3)
    self.heading = 0.
    self.goal = np.zeros(3)
    self.sink_strength   = 0
    self.source_strength   = 0.95


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

    cmd = 'rc {} {} {} {}'.format(
      clamp100(int(-V_err_xyz[1]*k)), # left_right_velocity
      clamp100(int(V_err_xyz[0]*k)),  # forward_backward_velocity
      clamp100(int(V_err_xyz[2]*k)),  # up_down_velocity
      clamp100(int(-err_heading*k)))  # yaw_velocity

    return(cmd)


#------------------------------------------------------------------------------
class Thread_mission(threading.Thread):

  def __init__(self,commands,targetId,rigidBodyDict,vehicles):
    threading.Thread.__init__(self)
    self.commands = commands
    self.targetId = targetId
    self.rigidBodyDict = rigidBodyDict
    self.vehicles = vehicles
    self.running = True

  def run(self):
    time.sleep(1)
    self.commands.put(('takeoff',))
    time.sleep(7)
    self.guidanceLoop()
    self.commands.put(('land',))

  def stop(self):
    self.running = False


  #--------------------------------------------------
  def compute_flow(self,vehicles):

    flow_vels = np.zeros([len(vehicles),3])

    V_sink    = np.zeros([len(vehicles),2]) # Velocity induced by sink element
    V_source  = np.zeros([len(vehicles),2]) # Velocity induced by source elements
    V_sum     = np.zeros([len(vehicles),2]) # V_gamma + V_sink + V_source
    V_normal  = np.zeros([len(vehicles),2]) # Normalized velocity
    V_flow    = np.zeros([len(vehicles),2]) # Normalized velocity inversly proportional to magnitude
    V_norm    = np.zeros([len(vehicles),1]) # L2 norm of velocity vector

    W_flow    = np.zeros([len(vehicles),1]) # Vertical velocity component (to be used in 3-D scenarios)

    for f,vehicle in enumerate(self.vehicles):

      # Cartesian velocity reprsentation by 2D sink
      # (Velocity induced by 2D point sink, eqn. 10.2 & 10.3 in Katz & Plotkin:)
      V_sink[f,0] = (-vehicle.sink_strength*(vehicle.position[0]-vehicle.goal[0]))/\
                    (2*np.pi*((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2))
      V_sink[f,1] = (-vehicle.sink_strength*(vehicle.position[1]-vehicle.goal[1]))/\
                    (2*np.pi*((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2))

      othervehicleslist = vehicles[:f] + vehicles[f+1:]
      for othervehicle in othervehicleslist:
        # Cartesian velocity reprsentation by 2D source 
        V_source[f,0] += (othervehicle.source_strength*(vehicle.position[0]-othervehicle.position[0]))/\
                         (2*np.pi*((vehicle.position[0]-othervehicle.position[0])**2+(vehicle.position[1]-othervehicle.position[1])**2))
        V_source[f,1] += (othervehicle.source_strength*(vehicle.position[1]-othervehicle.position[1]))/\
                         (2*np.pi*((vehicle.position[0]-othervehicle.position[0])**2+(vehicle.position[1]-othervehicle.position[1])**2))

      # Total velocity induced :
      V_sum[f,0] = V_sink[f,0] + V_source[f,0]
      V_sum[f,1] = V_sink[f,1] + V_source[f,1]

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

  #--------------------------------------------------

  def guidanceLoop(self):
    telloPeriod = 1/telloFreq
   
    loop_incr = 0
    unvalidcpt = 0
    while (self.running and loop_incr < 1500) :
      loop_incr = loop_incr+1
      time.sleep(telloPeriod)

      if not self.rigidBodyDict[self.targetId].valid:
        unvalidcpt = unvalidcpt+1
        if unvalidcpt == 10: break
        else: continue
      else: unvalidcpt= 0

      targetPos = self.rigidBodyDict[self.targetId].position
      for v in self.vehicles:
        v.update(self.rigidBodyDict[v.ID].position,
                self.rigidBodyDict[v.ID].velocity,
                self.rigidBodyDict[v.ID].heading,
                targetPos,
                5)

      flow_vels = self.compute_flow(self.vehicles)

      for i,v in enumerate(self.vehicles):
        cmd=v.apply_flow(flow_vels[i])
        print(cmd)
        self.commands.put((cmd,v.ID))


#------------------------------------------------------------------------------
def initNetDrone():
  telloDic = {}
  cpt=0
  for i in tellos_selected:
    if i in tellos_routeur:
      addr = ('192.168.1.'+str(i),8889)
      p = subprocess.Popen(["ping", "-q", "-c", "1", addr[0]], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      if p.wait() == 0:
        telloDic[i] = (tellos_routeur[i],addr)
        cpt = cpt + 1

    elif i in tellos_docker:
      telloDic[i] = tellos_docker[i]
      from dockernet import getdockeraddr   # Should run even without docker installed,
      ret,addr = getdockeraddr(telloDic[i]) # for non docker tellos
      if ret:
        telloDic[i] = (tellos_docker[i],addr)
        cpt = cpt + 1

  ret = True if cpt == len(tellos_selected) else False
  return(ret,telloDic)

#------------------------------------------------------------------------------
def main(telloNet):

  vehicleList = [];
  rigidBodyDict = {};
  rigidBodyDict[acTarg[0]] = Rigidbody(acTarg[0])
  for ac in tellos_selected:
    vehicleList.append(Vehicle(ac))
    rigidBodyDict[ac]=Rigidbody(ac)

  threadMotion = Thread_natnet(rigidBodyDict,optiFreq)
  threadMotion.start()

  commands = queue.Queue()
#  commands.put(('command',65))
  commands.put(('command',))

  threadMission = Thread_mission(commands,acTarg[0],rigidBodyDict,vehicleList)
  threadMission.start()

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

  try:
    while True:
      vtupple=commands.get()
      print(vtupple)
      if (len(vtupple)==2):
        sock.sendto(vtupple[0].encode(encoding="utf-8"),telloNet[vtupple[1]][1])
      else:
        for ac in telloNet:
          sock.sendto(vtupple[0].encode(encoding="utf-8"),telloNet[ac][1])

  except KeyboardInterrupt:
    print("\nWe are interrupting the program\n")
    time.sleep(1)
    threadMission.stop()
    threadMotion.stop()

    for ac in telloNet:
      sock.sendto('land'.encode(encoding="utf-8"),telloNet[ac][1])

    sock.close()
    print("mainloop stopped")


#------------------------------------------------------------------------------
if __name__=="__main__":

  ret,telloNet = initNetDrone()
  if ret: main(telloNet)
