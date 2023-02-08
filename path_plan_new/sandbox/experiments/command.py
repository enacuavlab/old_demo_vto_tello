#!/usr/bin/python3

import socket
import queue
import time
import threading

#------------------------------------------------------------------------------
class Thread_commandReal(threading.Thread):

  def __init__(self,quitflag,commands,droneReal):
    threading.Thread.__init__(self)
    self.quitflag = quitflag
    self.commands = commands
    self.droneReal = droneReal
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

  def run(self):
    try: 
      while not self.quitflag:
       vtupple=self.commands.get()
       print(vtupple)

       if (len(vtupple)==2):
         self.sock.sendto(vtupple[0].encode(encoding="utf-8"),self.droneReal[vtupple[1]][1])
       else:
        for ac in self.droneReal:
          self.sock.sendto(vtupple[0].encode(encoding="utf-8"),self.droneReal[ac][1])

    finally: 
      print("Thread_command stop")
