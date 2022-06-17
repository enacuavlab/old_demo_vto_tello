#!/usr/bin/python3
from path_plan_w_panel import ArenaMap, Flow_Velocity_Calculation
from vehicle import Vehicle
from natnet2python import Natnet2python
from natnet2python import Rigidbody
import numpy as np
import socket
import select
import threading
import time
import queue
import subprocess
import docker
#import pygame
#from djitellopy import TelloSwarm

#ac_list = [['TELLO-F0B594',65,0,0,0],]
#ac_list = [['TELLO-ED4310',60,0,0,0],]
ac_list = [['TELLO-F0B594',65,0,0,0],['TELLO-ED4310',60,0,0,0]]
ac_target = ['888','888']

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class Thread_batt(threading.Thread):
  def __init__(self,inout):
    threading.Thread.__init__(self)
    self.inout = inout
    self.running = True

  def run(self):
    while self.running:
      try:
        ready_read, ready_write, exceptional = select.select(self.inout,[],[],None)
        for ready in ready_read: 
          if self.running:
            data, server = ready.recvfrom(1024)
            batt=data.decode(encoding="utf-8")
            #for j in ac_list:
              #if(server[1] == j[3]):print(j[0]+" batt="+batt)
      except socket.timeout:
        pass

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class Thread_mission(threading.Thread):
  def __init__(self,commands,rigidbodies,vehicles,arena):
    threading.Thread.__init__(self)
    self.commands = commands
    self.rigidbodies = rigidbodies
    self.vehicles = vehicles
    self.arena = arena
    self.running = True

  def run(self):
    target_pos = np.zeros(3)

    for i in range(5):
      if self.running:time.sleep(1)

    if self.running: self.commands.put(('takeoff',))
    for i in range(5):
      if self.running:time.sleep(1)

    for i in range(60):
      if self.running:time.sleep(0.1)

      for r in self.rigidbodies:
        if r.ac_id == '888':
          target_pos = r.position

      for v in self.vehicles: v.Set_Goal(target_pos,5,0.0)

      for r in self.rigidbodies:
        for v in self.vehicles:
          if r.ac_id == v.ID: 
            v.update(r.position,r.velocity,r.heading)

      flow_vels = Flow_Velocity_Calculation(self.vehicles,self.arena)
      for i, vehicle in enumerate(self.vehicles):
        norm = np.linalg.norm(flow_vels[i])
        flow_vels[i] = flow_vels[i]/norm
        limited_norm = np.clip(norm,0., 0.8)
        fixed_speed = 0.3
        vel_enu = flow_vels[i]*limited_norm
        heading = np.arctan2(target_pos[1]-v.position[1],target_pos[0]-v.position[0])
        v.Set_Desired_Velocity(vel_enu, method='None')
        self.commands.put(v.send_velocity_enu(v.velocity_desired, heading),v.sock)

    if self.running: self.commands.put(('land',))
    print("Thread mission stopped")


#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
def init():
  ret=True
  for j in ac_list:
    for i in docker.DockerClient().containers.list():
      if (j[0] == i.name):
        res = subprocess.run(['docker','exec',i.name,'/bin/ping','-c 1','-W 1','192.168.10.1'],capture_output=True,text=True)
        if ("100% packet loss" in res.stdout):break
        print(i.name+" connected")
        res = subprocess.run(['docker','exec',i.name,'/usr/bin/env'],capture_output=True,text=True)
        tmp=res.stdout
        left="CMD_PORT="
        if (left in tmp):
          j[2]=(docker.DockerClient().containers.get(i.name).attrs['NetworkSettings']['IPAddress'])
          j[3]=int((tmp[tmp.index(left)+len(left):]).split()[0])
  for j in ac_list: 
    if ((j[2]==0) or (j[3]==0)): ret=False
  return(ret)

#------------------------------------------------------------------------------
def main():

  print("Matrix computing, wait 13sec ...")
  arena = ArenaMap(version = 65)
  arena.Inflate(radius = 0.2)
  arena.Panelize(size=0.01)
  arena.Calculate_Coef_Matrix()
  print("Matrix Computed")

  ac_id_list = [[str(_[1]),str(_[1])] for _ in ac_list]

  inout = []
  rigidbodies = [];vehicles = []
  rigidbodies.append(Rigidbody(str(ac_target[1])))

  for i in ac_list: 
    i[4]=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    i[4].bind(('172.17.0.1',i[3]))
    inout.append(i[4])
    rigidbodies.append(Rigidbody(str(i[1])))
    vehicles.append(Vehicle(str(i[1]),i[4]))


  vehicle_goto_goal_list =[[1.4,0,0,0] ] # altitude,AoA,t_start,Vinf=0.5,0.5,1.5
  for v in vehicles:
    v.Go_to_Goal(vehicle_goto_goal_list[0][0],vehicle_goto_goal_list[0][1],vehicle_goto_goal_list[0][2],vehicle_goto_goal_list[0][3])

  threadOpt = Natnet2python(ac_id_list + [ac_target], rigidbodies, freq=40)
  threadOpt.run()

  threadBatt = Thread_batt(inout)
  threadBatt.start()

  commands = queue.Queue()
  threadMission = Thread_mission(commands,rigidbodies,vehicles,arena)
  threadMission.start()

  commands.put(('command',))
  commands.put(('streamon',))
  commands.put(('downvision 0',))

  try:
    while True:
      while not commands.empty():
        vtupple=commands.get()
        print("-------")
        print(vtupple)
        print("-------")
#        if (len(vtupple)==2): 
#            vtupple[1].sendto(vtupple[0].encode(encoding="utf-8"),(j[2],j[3]))
#            print("unicast ",vtupple[0])
#        else: 
#            if(len(vtupple)==1): 
#              for j in ac_list: j[4].sendto(vtupple[0].encode(encoding="utf-8"),(j[2],j[3]))
#              print("vcast ",vtupple[0])

      time.sleep(0.1)


  except KeyboardInterrupt:
    print("\nWe are interrupting the program\n")
    time.sleep(1)
    threadMission.running = False
    threadBatt.running = False
    threadOpt.stop()
    for j in ac_list: j[4].sendto("land".encode(encoding="utf-8"),(j[2],j[3]))
    time.sleep(1)
    for j in ac_list: j[4].close()
    print("mainloop stopped")

#------------------------------------------------------------------------------
if __name__=="__main__":
  if(init()):main()
