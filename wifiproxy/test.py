#!/usr/bin/python3
import queue
import socket
import threading
import time
import docker
import subprocess
import sys
import re


#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class thread_ping(threading.Thread):
  def __init__(self,name,commands):
    threading.Thread.__init__(self)
    self.name = name
    self.commands = commands
    self.running = True

  def run(self):
    prev_state=False
    while self.running:
      res = subprocess.run(
        ['docker','exec',self.name,'/bin/ping','-c 1','-W 1','192.168.10.1'], capture_output=True, text=True
      )
      tmp=res.stdout
      if ("100% packet loss" in tmp):curr_state=False
      else: curr_state=True
      if ((prev_state != curr_state) and curr_state): self.startup_sequence()
      prev_state = curr_state

    print("Thread ping stopped")

  def startup_sequence(self):
    for i in range(5): 
      if self.running: time.sleep(1)
    if self.running:
      self.commands.put('command')
      self.commands.put('streamon')
      self.commands.put('downvision 0')


#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class thread_monitor_batt(threading.Thread):
  def __init__(self,sock,commands):
    threading.Thread.__init__(self)
    self.sock = sock
    self.commands = commands
    self.running = True

  def run(self):
    msg='battery?'
    while self.running:
      if (msg not in self.commands.queue): self.commands.put(msg)
      try:
        data, server = self.sock.recvfrom(1518)
        tmp=data.decode(encoding="utf-8")
        if(tmp.count('\n')==1):
          batt=tmp[:-1]
          print(batt)
          time.sleep(1)

      except socket.timeout:
        pass

    print("Thread batt stopped")


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
    if self.running: self.commands.put('land')
    print("Thread mission stopped")


#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
def check_connection(name):
  ret=False
  res1 = subprocess.run(
    ['docker','exec',name,'/bin/bash','-c','iwconfig | grep '+name], capture_output=True, text=True
  )
  if(len(res1.stdout)>0):
    wifidev = res1.stdout.split()[0]
    res2 = subprocess.run(
      ['docker','exec',name,'/bin/bash','-c','ip a | grep '+wifidev], capture_output=True, text=True
    )
    tmp=res2.stdout
    left="inet"
    if (left in tmp):
      ipwifi=(tmp[1+tmp.index(left)+len(left):tmp.index("/")])
      if (re.match(r'^(?:[0-9]{1,3}\.){3}[0-9]{1,3}$', ipwifi)):
        res3 = subprocess.run(
          ['docker','exec',name,'/bin/ping','-c 1','-W 1','192.168.10.1'], capture_output=True, text=True
        )
        tmp=res3.stdout
        if ("100% packet loss" not in tmp):ret=True
  return(ret)


def close_connection(name):
  res = subprocess.run(
    ['docker','exec',name,'iw dev $WIFI_DEV disconnect'], capture_output=True, text=True
  )


#------------------------------------------------------------------------------
if __name__ == '__main__':

  if(len(sys.argv)==2):
    if(sys.argv[1] == '?'):
      for i in  docker.DockerClient().containers.list():
        print(i.name+" created")
        if(check_connection(i.name)): print(i.name+" connected")
    else:
      for i in  docker.DockerClient().containers.list():
        if(sys.argv[1] == i.name):
          if(check_connection(i.name)):

            print(i.name+" created & connected")
            tello_add = (docker.DockerClient().containers.get(i.name).attrs['NetworkSettings']['IPAddress'], 8889)
            print(tello_add)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(1.0)
            commands = queue.Queue()
            commands.put('command')
#            threadPing = thread_ping(i.name,commands)
            threadBatt = thread_monitor_batt(sock,commands)
#            threadMission = thread_mission(commands)
#            threadPing.start()
#            threadBatt.start()
#            threadMission.start()

            try:
              while True:
                while not commands.empty():
#                  print(list(commands.queue))
                  msg=commands.get()
#                  print("Sending <"+msg+">")
#                  sock.sendto(msg.encode(encoding="utf-8"),tello_add)

                time.sleep(0.1)

            except KeyboardInterrupt:
              print("\nWe are interrupting the program\n")
#              threadMission.running = False
              threadBatt.running = False
#              threadPing.running = False
              time.sleep(1)
              sock.close()
#              close_connection(i.name)
              print("mainloop stopped")
