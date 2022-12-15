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

tellos_selected = (65,)
telloFreq = 10

optiFreq = 20 # Check that optitrack stream at least with this value

#------------------------------------------------------------------------------
class Rigidbody(): # should be compliant with Thread_natnet 

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
    self.position_enu = np.zeros(3)
    self.velocity_enu = np.zeros(3)
    self.heading = 0.


  def update(self,position,velocity,heading):
    self.position_enu = position
    self.velocity_enu = velocity
    angle = heading - np.pi / 2
    self.heading = -np.arctan2(np.sin(angle), np.cos(angle))


  def fly_to_enu(self,position_enu,heading=None):
    if heading is None:
      heading = self.heading
    pos_error = position_enu - self.position_enu
    vel_enu = pos_error*1.2 - self.velocity_enu

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

  def __init__(self,commands,rigidBodyDict,vehicles):
    threading.Thread.__init__(self)
    self.commands = commands
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

  def guidanceLoop(self):
    telloPeriod = 1/telloFreq
   
    goal_enu = np.array([1.0,1.0,1.0])

    loop_incr = 0
    while (self.running and loop_incr < 1500) :
      loop_incr = loop_incr+1
      time.sleep(telloPeriod)

      for v in self.vehicles:
        v.update(self.rigidBodyDict[v.ID].position,self.rigidBodyDict[v.ID].velocity,self.rigidBodyDict[v.ID].heading)

      for v in self.vehicles:
   
        heading = np.arctan2(goal_enu[1]-v.position_enu[1],goal_enu[0]-v.position_enu[0])
        cmd=v.fly_to_enu(goal_enu,heading)
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
  for ac in tellos_selected:
    vehicleList.append(Vehicle(ac))
    rigidBodyDict[ac]=Rigidbody(ac)

  threadMotion = Thread_natnet(rigidBodyDict,optiFreq)
  threadMotion.start()

  commands = queue.Queue()
  commands.put(('command',))

  threadMission = Thread_mission(commands,rigidBodyDict,vehicleList)
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
