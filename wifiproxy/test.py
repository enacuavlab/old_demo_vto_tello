#!/usr/bin/python3
from NatNetClient import NatNetClient
from collections import deque

from path_plan_w_panel import ArenaMap, Flow_Velocity_Calculation
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


#acDict = {60:[('TELLO-ED4310')],65:[('TELLO-F0B594')]}
#acDict = {60:[('TELLO-ED4310')]}
acDict = {65:[('TELLO-F0B594')]}
acTarg = [888,'Helmet']

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class Rigidbody():
  def __init__(self,ac_id):
    self.ac_id = ac_id
    self.position = np.zeros(3)
    self.velocity = np.zeros(3)
    self.heading = 0.
    self.quat = np.zeros(4)


class Natnet2python():
  def __init__(self, rigidBodyDict, freq=20, server="127.0.0.1", dataport=int(1511), commandport=int(1510), vel_samples=int(4), verbose=False):
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
      if not valid: continue # skip if rigid body is not valid
      if ac_id not in self.rigidBodyDict: continue
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
            print(str(servDict[server[1]])+" batt:"+str(batt))
      except socket.timeout:
        pass

  def stop(self):
    self.running = False

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
def init(inoutDict):
  ret=True
  for key in acDict.keys(): 
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
  for key in acDict.keys(): 
    if(len(inoutDict[key]))==1: ret=False
  return(inoutDict,ret)

#------------------------------------------------------------------------------
def main(inoutDict):
#  print(ac_dict.keys())
#  print(ac_dict.values())
#  print(ac_dict.get(65))
#  ac_dict.update({65:(,9091)})
#  ac_dict[65].append((100))
#  print(ac_dict[65])
#  ac_dict[65].append((200))
#  print(ac_dict[65][2])

#  print("Matrix computing, wait 13sec ...")
#  arena = ArenaMap(version = 65)
#  arena.Inflate(radius = 0.2)
#  arena.Panelize(size=0.01)
#  arena.Calculate_Coef_Matrix()
#  print("Matrix Computed")

  vehicleList = [];
  rigidBodyDict = {};
  rigidBodyDict[acTarg[0]] = Rigidbody(acTarg[0])
  for ac in acDict:
    sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('172.17.0.1',inoutDict[ac][1]))
    inoutDict[ac]=((sock,inoutDict[ac][0],inoutDict[ac][1]))
    rigidBodyDict[ac]=Rigidbody(ac)
    vehicleList.append(Vehicle(ac))

  threadBatt = Thread_batt(inoutDict)
  threadBatt.start()

#  threadOpt = Natnet2python(rigidBodyDict,freq=20)
#  threadOpt.run()

#  flow_vels = Flow_Velocity_Calculation(vehicles,arena)

  commands = queue.Queue()
  try:
    while True:
      while not commands.empty():
        msg=commands.get()
      time.sleep(0.1)

  except KeyboardInterrupt:
    print("\nWe are interrupting the program\n")
    time.sleep(1)
    threadBatt.stop()
#    threadOpt.stop()
    for ac in acDict: inoutDict[ac][0].close()
    print("mainloop stopped")


#------------------------------------------------------------------------------
if __name__=="__main__":
  print("---------------------------------------")
  print("Tello will connected within 14 seconds")
  print("if tello led do not blink pink, restart it !")
  print("---------------------------------------")
  inoutDict = {}
  inout, ret = init(inoutDict)
  if(ret):main(inoutDict)
