#!/usr/bin/python3

from natnet import Thread_natnet
from mission import Thread_mission
from vehicle import Vehicle
from buildingOut import BuildingOut

import numpy as np
import json
import sys
import argparse
import subprocess
import queue
import socket
import time

#tellos_routeur = {61:'TELLO-ED433E',62:'TELLO-ED4317',63:'TELLO-ED42A3',64:'TELLO-ED4381'}
#tellos_docker = {60:'TELLO-ED4310',65:'TELLO-F0B594',66:'TELLO-99CE21'}

tellos_routeur = {61:'TELLO-ED433E',62:'TELLO-ED4317',63:'TELLO-ED42A3',64:'TELLO-ED4381',65:'TELLO-F0B594',66:'TELLO-99CE21'}
tellos_docker = {60:'TELLO-ED4310',67:'TELLO-99CE5A',68:'TELLO-99CE4E'}
#tellos_docker = {67:'TELLO-99CE5A',68:'TELLO-99CE4E'}

#------------------------------------------------------------------------------
#tellos_selected = (60,)
tellos_selected = (65,)
#tellos_selected = (66,)
#tellos_selected = (67,)
#tellos_selected = (68,)
#tellos_selected = (65,66,)
#tellos_selected = (67,68,)
#tellos_selected = (66,67,68,)
#tellos_selected = (65,66,67,68,)

acTarg = [888,'Helmet']

sourceStrength = 0.95 # Tello repelance

optiFreq = 20 # Check that optitrack stream at least with this value 

#--------------------------------------------------------------------------------
#
# ./exec_run.py -i material/outputfromtake.json
# or
# ./exec_run.py -i material/outputfromnatnet.json
#
#--------------------------------------------------------------------------------

# This program compute and send in realtime flight guidance to vehicles
# - to track a goal
# - to avoid buildings
# - to avoid vehicles

#------------------------------------------------------------------------------
class ArenaMap():
  def __init__(self):
    self.panels = None
    self.wind = [0,0]
    self.windT = 0
    self.buildings = []


#------------------------------------------------------------------------------
class Rigidbody():
  def __init__(self,ac_id):
    self.ac_id = ac_id
    self.valid = False
    self.position = np.zeros(3)
    self.velocity = np.zeros(3)
    self.heading = 0.
    self.quat = np.zeros(4)


#------------------------------------------------------------------------------
def initNetDrone():
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



def initArena(jsonfile):
  retmat = {}
  with open(jsonfile, "r") as infile: retmat = json.load(infile)
  infile.close()
  arena = ArenaMap()
  for val0, val1, val2, val3, val4, val5 in retmat.values():
    b = BuildingOut(val0,np.array(val1))
    b.vertices = np.array(val1) # udpate vertices
    b.pcp = np.array(val2)
    b.pb = np.array(val3)
    b.nop = val4
    b.K_inv = np.array(val5)
    arena.buildings.append(b)
  return arena


#------------------------------------------------------------------------------
def main(arena,telloNet):

  vehicleList = [];
  rigidBodyDict = {};
  rigidBodyDict[acTarg[0]] = Rigidbody(acTarg[0])
  for ac in tellos_selected:
    vehicleList.append(Vehicle(ac,sourceStrength))
    rigidBodyDict[ac]=Rigidbody(ac)

  threadMotion = Thread_natnet(rigidBodyDict,optiFreq)
  threadMotion.start()

  commands = queue.Queue()
  commands.put(('command',))
#  commands.put(('streamon',66))
#  commands.put(('downvision 0',66))
  commands.put(('streamon',))
  commands.put(('downvision 0',))

  threadMission = Thread_mission(commands,acTarg[0],rigidBodyDict,vehicleList,arena)
  threadMission.start()

  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

  print("READY")

  try:
    while True:
      vtupple=commands.get()
#      print(vtupple)
      if (len(vtupple)==2):
        sock.sendto(vtupple[0].encode(encoding="utf-8"),telloNet[vtupple[1]][1])
      else:
        for ac in telloNet:
          sock.sendto(vtupple[0].encode(encoding="utf-8"),telloNet[ac][1])

  except KeyboardInterrupt:
    print("\nWe are interrupting the program\n")
    time.sleep(1)
    threadMission.stop()
    threadMotion.stop()
    sock.close()
    print("mainloop stopped")

#------------------------------------------------------------------------------
if __name__=="__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument( '-i', '--input_jsonmatrix')
  args = parser.parse_args()

  if (args.input_jsonmatrix):
    ret,telloNet = initNetDrone()
    if ret:
      arena = initArena(args.input_jsonmatrix)
      main(arena,telloNet)
