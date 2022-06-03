#!/usr/bin/python3
import socket
import time
import threading
import queue

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
          print(batt)
          self.sockGCS.sendto(batt.encode(encoding="utf-8"),self.addGCS)
          time.sleep(1)

      except .encode(encoding="utf-8")socket.timeout:
        pass

    print("Thread batt stopped")


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
        print(tmp)

      except socket.timeout:
        pass

    print("Thread command stopped")


#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
if __name__ == '__main__':

  sockGCS = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  addGCS = ('172.17.0.1',8889)

  sockDrone = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  addDrone = ('192.168.10.1',8889)
  sockDrone.bind (('192.168.10.2',8889))
  sockDrone.settimeout(1.0)

  commands = queue.Queue()
  commands.put('command')

  threadBatt = thread_monitor_batt(addGCS,sockGCS,sockDrone,commands)
  threadCmd = thread_command(sockGCS,commands)
  threadBatt.start()

  try:
    while True:
      while not commands.empty():
        print(list(commands.queue))
        msg=commands.get()
        print("Sending <"+msg+">")
        sockDrone.sendto(msg.encode(encoding="utf-8"),addDrone)

      time.sleep(0.1)

  except KeyboardInterrupt:
    threadCmd.running = False
    threadBatt.running = False
    time.sleep(1)
    sockDrone.close()
    sockGCS.close()
    print("mainloop stopped")

