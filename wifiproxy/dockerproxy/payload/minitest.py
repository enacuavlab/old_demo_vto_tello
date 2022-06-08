#!/usr/bin/python3
import socket
import time
import threading
import queue
import sys

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class thread_monitor_batt(threading.Thread):
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
          time.sleep(1)

      except socket.timeout:
        pass


#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class thread_command(threading.Thread):
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
def main(cmd_port):

  sockGCS = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  addGCS = ('172.17.0.1',cmd_port)
  sockGCS.bind(('172.17.0.2',cmd_port))
  sockGCS.settimeout(1.0)

  sockDrone = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  addDrone = ('192.168.10.1',8889)
  sockDrone.bind (('192.168.10.2',8889))
  sockDrone.settimeout(1.0)

  commands = queue.Queue()
  commands.put('command')

  threadBatt = thread_monitor_batt(addGCS,sockGCS,sockDrone,commands)
  threadCmd = thread_command(sockGCS,commands)
  threadBatt.start()
  threadCmd.start()

  try:
    while True:
      while not commands.empty():
        msg=commands.get()
        sockDrone.sendto(msg.encode(encoding="utf-8"),addDrone)

      time.sleep(0.1)

  except KeyboardInterrupt:
    threadCmd.running = False
    threadBatt.running = False
    time.sleep(1)
    sockDrone.close()
    sockGCS.close()

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
if __name__ == '__main__':

  if(len(sys.argv)==2):main(int(sys.argv[1]))
