#!/usr/bin/python3
import socket
import time
import threading
import queue
import sys
import subprocess
import docker

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class thread_startup(threading.Thread):
  def __init__(self,commands):
    threading.Thread.__init__(self)
    self.commands = commands
    self.running = True

  def run(self):
    if self.running:
#      self.commands.put('command')
      self.commands.put('streamon')
      self.commands.put('downvision 0')
    print("Thread startup stopped")


#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class thread_mission(threading.Thread):
  def __init__(self,commands):
    threading.Thread.__init__(self)
    self.commands = commands
    self.running = True

  def run(self):

    for i in range(5):
      if self.running:time.sleep(1)
    if self.running: self.commands.put('takeoff')

    for i in range(8):
      if self.running:time.sleep(1)
    if self.running: self.commands.put('up 100')

    for i in range(8):
      if self.running:time.sleep(1)
    if self.running: self.commands.put('land')

    print("Thread mission stopped")


#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class thread_batt(threading.Thread):
  def __init__(self,sock):
    threading.Thread.__init__(self)
    self.sock = sock
    self.running = True

  def run(self):
    print("Thread batt started")
    while self.running:
      try:
        data, server = self.sock.recvfrom(1518)
        batt=data.decode(encoding="utf-8")
        print(batt)

      except socket.timeout:
        pass

    print("Thread batt stopped")


#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
def main(docker_ip,cmd_port):

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.bind(('172.17.0.1',cmd_port))

  tello_add=(docker_ip,cmd_port)

  commands = queue.Queue()
  commands.put('command')

  threadBatt = thread_batt(sock)
  threadStart = thread_startup(commands)
  threadMission = thread_mission(commands)

  threadBatt.start()
  threadStart.start()
  threadMission.start()

  try:
    while True:
      while not commands.empty():
        msg=commands.get()
        print("Sending <"+msg+">")
        sock.sendto(msg.encode(encoding="utf-8"),tello_add)

      time.sleep(0.1)


  except KeyboardInterrupt:
    print("\nWe are interrupting the program\n")
    threadMission.running = False
    threadStart.running = False
    threadBatt.running = False
    time.sleep(1)
    sock.close()
    print("mainloop stopped")


#------------------------------------------------------------------------------
if __name__ == '__main__':

  if(len(sys.argv)==2):
    if(sys.argv[1] == '?'):
      for i in  docker.DockerClient().containers.list():
        print(i.name+" created")
    else:
      for i in  docker.DockerClient().containers.list():
        if(sys.argv[1] == i.name):
          res = subprocess.run(
            ['docker','exec',i.name,'/bin/ping','-c 1','-W 1','192.168.10.1'], capture_output=True, text=True
          )
          if ("100% packet loss" in res.stdout):break
          print(i.name+" connected")
          res = subprocess.run(
            ['docker','exec',i.name,'/usr/bin/env'], capture_output=True, text=True
          )
          tmp=res.stdout
          left="CMD_PORT="
          if (left in tmp):
            cmd_port=int((tmp[tmp.index(left)+len(left):]).split()[0])
            docker_ip=(docker.DockerClient().containers.get(i.name).attrs['NetworkSettings']['IPAddress'])
            print(docker_ip)
            print(cmd_port)
            main(docker_ip,cmd_port)
