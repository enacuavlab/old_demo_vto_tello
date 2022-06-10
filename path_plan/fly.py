#!/usr/bin/python3
from natnet2python import Natnet2python
from natnet2python import Vehicle
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

#ac_list = [['TELLO-F0B594',59,0,0,0],]
ac_list = [['TELLO-ED4310',60,0,0,0],]
#ac_list = [['TELLO-F0B594',59,0,0,0],['TELLO-ED4310',60,0,0,0]]

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class Tello():
  def __init__(self,ac_id):
    self.ac_id = ac_id
    self.position_enu = np.zeros(3)
    self.velocity_enu = np.zeros(3)
    self.heading = 0.
    self.last_rc_control_timestamp = time.time()

  def update(self,position,velocity,heading):
    self.position_enu = position
    self.velocity_enu = velocity
    self.heading = heading

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

  def send_velocity_enu(self, vel_enu, heading):
    k = 100.
#    def RBI_pprz(psi):
#      cp = np.cos(psi)
#      sp = np.sin(psi)
#      return np.array([[sp, cp, 0.],
#                       [cp, -sp, 0.],
#                       [0., 0., 1.]])
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
      self.send_rc_control(int(-V_err_xyz[1]*k),int(V_err_xyz[0]*k),int(V_err_xyz[2]*k), int(-err_heading*k))

  def fly_to_enu(self,position_enu, heading=None):
    if heading is None:
      heading = self.heading
      pos_error = position_enu - self.position_enu
      vel_enu = pos_error*1.2 - self.velocity_enu
      return(self.send_velocity_enu(vel_enu, heading))


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
            for j in ac_list:
              if(server[1] == j[3]):print(j[0]+" batt="+batt)
      except socket.timeout:
        pass

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class Thread_mission(threading.Thread):
  def __init__(self,commands,vehicles,tellos):
    threading.Thread.__init__(self)
    self.commands = commands
    self.vehicles = vehicles
    self.tellos = tellos
    self.running = True

  def run(self):
    for v in self.vehicles:
      for t in self.tellos:
        if v.ac_id == t.ac_id: t.update(v.position,v.velocity,v.heading)

    print("in for")
    for t in self.tellos: print(t.fly_to_enu(np.array([0.5, 3.5, 1.4]),0.))
    print("out for")



#    for i in range(5):
#      if self.running:time.sleep(1)
#    if self.running: self.commands.put('takeoff')
#
#    for i in range(8):
#      if self.running:time.sleep(1)
#    if self.running: self.commands.put('up 100')
#
#    for i in range(8):
#      if self.running:time.sleep(1)
#    if self.running: 
#      print(fly_to_enu(np.array([0.5, 3.5, 1.4]),0.))
#        #self.commands.put('up 100')
#      
#
#    for i in range(8):
#      if self.running:time.sleep(1)
#    if self.running: self.commands.put('land')

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

  inout = []
  for j in ac_list:
    j[4]=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    j[4].bind(('172.17.0.1',j[3]))
    inout.append(j[4])

  ac_id_list = [[str(_[1]),str(_[1])] for _ in ac_list]

  vehicles = [];tellos = []
  for i in ac_list: 
    vehicles.append(Vehicle(str(i[1])))
    tellos.append(Tello(str(i[1])))

  threadOpt = Natnet2python(ac_id_list, vehicles, freq=40)
  threadOpt.run()

  threadBatt = Thread_batt(inout)
  threadBatt.start()

  commands = queue.Queue()
  threadMission = Thread_mission(commands,vehicles,tellos)
  threadMission.start()


  commands.put('command')
  commands.put('streamon')
  commands.put('downvision 0')

  try:
    while True:
      while not commands.empty():
        msg=commands.get()
        print("Sending <"+msg+">")
        for j in ac_list: j[4].sendto(msg.encode(encoding="utf-8"),(j[2],j[3]))

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

#  pygame.display.init()
#  pygame.joystick.init()
#  joystick_nr = pygame.joystick.get_count()
#  controller = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]
#  for j in range(pygame.joystick.get_count()): controller[j].init()
#
#  #ac_list = [['60', '60', docker.DockerClient().containers.get('TELLO-ED4310').attrs['NetworkSettings']['IPAddress']],]
#  ac_list = [['59', '59', docker.DockerClient().containers.get('TELLO-F0B594').attrs['NetworkSettings']['IPAddress']],
#             ['60', '60', docker.DockerClient().containers.get('TELLO-ED4310').attrs['NetworkSettings']['IPAddress']]]
#
#  ip_list = [_[2] for _ in ac_list]
##  swarm = TelloSwarm.fromIps(ip_list)
#  id_list = [_[1] for _ in ac_list]
##  for i,id in enumerate(id_list): swarm.tellos[i].set_ac_id(id)
#
##  swarm.connect(wait_for_state=False)
##  time.sleep(5)
##  swarm.takeoff()
##  time.sleep(10)
##  swarm.land()

#------------------------------------------------------------------------------
if __name__=="__main__":
  if(init()):main()
