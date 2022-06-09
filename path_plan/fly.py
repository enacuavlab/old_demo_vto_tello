#!/usr/bin/python3
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
ac_list = [['TELLO-F0B594',59,0,0,0],['TELLO-ED4310',60,0,0,0]]

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class thread_batt(threading.Thread):
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

  threadBatt = thread_batt(inout)
  threadBatt.start()

  commands = queue.Queue()
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
    threadBatt.running = False
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
