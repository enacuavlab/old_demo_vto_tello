#!/usr/bin/python3
import socket
import time
import threading
import queue
import sys

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class thread_batt(threading.Thread):
  def __init__(self,addGCS,sockGCS,sockDrone,commands):
    threading.Thread.__init__(self)
    self.addGCS = addGCS
    self.sockGCS = sockGCS
    self.sockDrone = sockDrone
    self.commands = commands
    self.running = True

  def run(self):
    msg='battery?'
    while self.running:
      if (msg not in self.commands.queue): self.commands.put(msg)
      try:
        data, server = self.sockDrone.recvfrom(1518)
        tmp=data.decode(encoding="utf-8")
        if(tmp.count('\n')==1):
          batt=tmp[:-1]
          self.sockGCS.sendto(batt.encode(encoding="utf-8"),self.addGCS)
          print(batt)
          time.sleep(1)

      except socket.timeout:
        pass

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class thread_gcs(threading.Thread):

  def __init__(self,sockGCS,commands):
    threading.Thread.__init__(self)
    self.sockGCS = sockGCS
    self.commands = commands
    self.running = True

  def run(self):
    while self.running:
      try:
        data, server = self.sockGCS.recvfrom(1518)
        tmp=data.decode(encoding="utf-8")
        self.commands.put(tmp)

      except socket.timeout:
        pass


#------------------------------------------------------------------------------
def main(docker_ip,cmd_port):

  sockDrone = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  addDrone = ('192.168.10.1',8889)
  sockGCS = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sockGCS.bind((docker_ip,cmd_port))

  commands = queue.Queue()
  commands.put('command')

  threadBatt = thread_batt(('172.17.0.1',cmd_port),sockGCS,sockDrone,commands)
  threadGCS = thread_gcs(sockGCS,commands)
  threadBatt.start()
  threadGCS.start()

  try:
    while True:
      while not commands.empty():
        msg=commands.get()
        sockDrone.sendto(msg.encode(encoding="utf-8"),addDrone)

      time.sleep(0.1)

  except KeyboardInterrupt:
    threadGCS.running = False
    threadBatt.running = False
    time.sleep(1)
    sockDrone.close()
    sockGCS.close()

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
if __name__ == '__main__':

  if(len(sys.argv)==3):main(sys.argv[1],int(sys.argv[2]))
