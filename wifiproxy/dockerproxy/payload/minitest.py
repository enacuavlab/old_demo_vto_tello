#!/usr/bin/python3
import socket
import time
import threading
import queue

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class thread_monitor_batt(threading.Thread):
  def __init__(self,sock,commands):
    threading.Thread.__init__(self)
    self.sock = sock
    self.commands = commands
    self.running = True

  def run(self):
    sockOut = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    msg='battery?'
    while self.running:
      if (msg not in self.commands.queue): self.commands.put(msg)
      try:
        data, server = self.sock.recvfrom(1518)
        tmp=data.decode(encoding="utf-8")
        if(tmp.count('\n')==1):
          batt=tmp[:-1]
          print(batt)
          sockOut.sendto(batt.encode(encoding="utf-8"),('172.17.0.2',8889))
          time.sleep(1)

      except socket.timeout:
        pass

    print("Thread batt stopped")


#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
if __name__ == '__main__':
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.settimeout(1.0)

  commands = queue.Queue()
  commands.put('command')

  threadBatt = thread_monitor_batt(sock,commands)
  threadBatt.start()

  try:
    while True:
      while not commands.empty():
        print(list(commands.queue))
        msg=commands.get()
        print("Sending <"+msg+">")
        sock.sendto(msg.encode(encoding="utf-8"),('192.168.10.1',8889))


      time.sleep(0.1)

  except KeyboardInterrupt:
    threadBatt.running = False
    time.sleep(1)
    sock.close()
    print("mainloop stopped")

