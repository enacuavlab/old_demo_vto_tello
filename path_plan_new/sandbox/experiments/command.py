#!/usr/bin/python3

import socket
import queue
import time
import threading

#------------------------------------------------------------------------------
class Thread_command(threading.Thread):

  def __init__(self,quitflag,commands):
    threading.Thread.__init__(self)
    self.quitflag = quitflag
    self.commands = commands
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

  def run(self):
    self.commandLoop()

  def commandLoop(self):
    try: 
      while not self.quitflag:
       vtupple=self.commands.get()
       print(vtupple)

    finally: 
      print("Thread_command stop")
