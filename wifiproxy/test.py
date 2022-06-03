#!/usr/bin/python3
import socket
import time
import threading

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
class thread_monitor_batt(threading.Thread):
  def __init__(self,sock):
    threading.Thread.__init__(self)
    self.sock = sock
    self.running = True

  def run(self):
    while self.running:
      try:
        data, server = self.sock.recvfrom(1518)
        tmp=data.decode(encoding="utf-8")
        if(tmp.count('\n')==1):
          batt=tmp[:-1]
          print(batt)

      except socket.timeout:
        pass

    print("Thread batt stopped")


#------------------------------------------------------------------------------
if __name__ == '__main__':

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.bind(('172.17.0.1',8889))
  sock.settimeout(1.0)

  threadBatt = thread_monitor_batt(sock)

  try:
    while True:
      time.sleep(0.1)


  except KeyboardInterrupt:
    print("\nWe are interrupting the program\n")
    threadBatt.running = False
    time.sleep(1)
    sock.close()
    print("mainloop stopped")
