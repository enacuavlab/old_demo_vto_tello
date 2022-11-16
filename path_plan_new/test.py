#!/usr/bin/python3

from natnet import Thread_natnet

import sys
import subprocess
import socket
import threading
import queue
import time

tellos_routeur = {61:'TELLO-ED433E',62:'TELLO-ED4317',63:'TELLO-ED42A3',64:'TELLO-ED4381'}
tellos_docker = {60:'TELLO-ED4310',65:'TELLO-F0B594',66:'TELLO-99CE21'}


#------------------------------------------------------------------------------
#tellos_selected = (64,65)
tellos_selected = (64,)

acTarg = [888,'Helmet']

telloFreq = 10

optiFreq = 20 # Check that optitrack stream at least this value 

#------------------------------------------------------------------------------
def initNet():
  telloDic = {}
  cpt=0
  for i in tellos_selected:
    if i in tellos_routeur:
      addr = ('192.168.1.'+str(i),8889)
      p = subprocess.Popen(["ping", "-q", "-c", "1", addr[0]], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      if p.wait() == 0:
        telloDic[i] = (tellos_routeur[i],addr)
        cpt = cpt + 1
  
    elif i in tellos_docker:
      telloDic[i] = tellos_docker[i]
      from dockernet import getdockeraddr   # Should run even without docker installed,
      ret,addr = getdockeraddr(telloDic[i]) # for non docker tellos
      if ret:
        telloDic[i] = (tellos_docker[i],addr)
        cpt = cpt + 1

  ret = True if cpt == len(tellos_selected) else False
  return(ret,telloDic)


#------------------------------------------------------------------------------
class Thread_mission(threading.Thread):

  def __init__(self,commands):
    threading.Thread.__init__(self)
    self.running = True
    self.commands = commands


  def run(self):
    time.sleep(1)
    self.commands.put(('takeoff',))
    time.sleep(7)
    self.commands.put(('land',))

  def stop(self):
    self.running = False


#------------------------------------------------------------------------------
def main():

  ret,telloDic = initNet()
  if ret == False: sys.exit(-1)

  rgbDict = {'888':None,'65':None,'60':None}

  threadMotion = Thread_natnet(optiFreq,rgbDict)
  threadMotion.start()

  commands = queue.Queue()
  commands.put(('command',))
  commands.put(('streamon',))
  commands.put(('downvision 0',))

  threadMission = Thread_mission(commands)
  threadMission.start()

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

  try:
    while True:
      vtupple=commands.get()
      if (len(vtupple)==2):
        sock.sendto(vtupple[0].encode(encoding="utf-8"),vtupple[1])
      else:
        for ac in telloDic:
          sock.sendto(vtupple[0].encode(encoding="utf-8"),telloDic[ac][1])

  except KeyboardInterrupt:
    print("\nWe are interrupting the program\n")
    time.sleep(1)
    threadMission.stop()
    threadMotion.stop()
    sock.close()
    print("mainloop stopped")

#------------------------------------------------------------------------------
if __name__=="__main__":
  main()
