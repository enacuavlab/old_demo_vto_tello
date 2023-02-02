#!/usr/bin/python3

from natnet4 import Rigidbody,Thread_natnet
from mission import Thread_mission
from command import Thread_commandReal
from simulation import Thread_commandSim
from vehicle import Vehicle
from netdrone import initNetDrone
from drawingGL import DrawingGL
#from drawing3D import Drawing3D
#from drawing2D import Drawing2D

import argparse
import time

import threading,queue
import numpy as np

#------------------------------------------------------------------------------
#
# ./exec_1.py --as 45
# ./exec_1.py --as 45[-1.1,-2.2,3.3]
# ./exec_1.py --as 45[-1.1,-2.2,3.3][1.1,2.2,3.3]
#
#------------------------------------------------------------------------------
acTarg = [888,'Helmet']

optiFreq = 20 # Check that optitrack stream at least with this value

FPS = 30

#------------------------------------------------------------------------------
class Flag(threading.Event):
  def __bool__(self):
    return self.is_set()

#------------------------------------------------------------------------------
def main(droneReal,droneSim):

  vehicleListReal = []
  rigidBodyDict = {}
  rigidBodyDict[acTarg[0]] = Rigidbody(acTarg[0])
  for ac in droneReal:
    vehicleListReal.append(Vehicle(ac))
    rigidBodyDict[ac]=Rigidbody(ac)

  vehicleListSim = []
  for i,elt in droneSim.items():
    vel = Vehicle(i)
    vel.position = elt
    vehicleListSim.append(vel)

  flag = Flag()

  if vehicleListReal:
    try:
      threadMotion = Thread_natnet(flag,rigidBodyDict,optiFreq)
      threadMotion.start()
    except ValueError as msg:
      print(msg)
      exit()

    commands = queue.Queue()
  
    threadMission = Thread_mission(flag,rigidBodyDict,acTarg[0])
    threadMission.start()
  
    threadCmdReal = Thread_commandReal(flag,commands)
    threadCmdReal.start()

  if vehicleListSim:
    threadCmdSim = Thread_commandSim(flag,vehicleListSim)
    threadCmdSim.start()
      

  try:
    DrawingGL(FPS,vehicleListSim,rigidBodyDict).start()

  except KeyboardInterrupt:
    print("KeyboardInterrupt")
    flag.set()

  finally:
    print("finally")
    flag.set()
    if vehicleListSim:
      threadCmdSim.join()
    if vehicleListReal: 
      threadMotion.join()
      threadMission.join()
      threadCmdReal.join()


#------------------------------------------------------------------------------
def argsforSim(param):
  global droneSim
  for elt in param.split():
    if '[' in elt:
      val=elt.split('[')
      pos=np.fromstring((val[1][:-1]), dtype=float, sep=',')
      droneSim[int(val[0])]=pos
    else:
      droneSim[int(elt)]=np.zeros([1, 3], dtype = float)

#------------------------------------------------------------------------------
if __name__=="__main__":
  droneSim = {}  
  parser = argparse.ArgumentParser()
  parser.add_argument('--ar', nargs='+', dest='realacs', type=int)
  parser.add_argument('--as', nargs='+', type=argsforSim)
  args = parser.parse_args()

  ret=True
  droneReal = {}
  if args.realacs is not None:
    ret = False
    ret,droneAddrs = initNetDrone(args.realacs)
    if ret: droneReal = droneAddrs
  if ret:
    main(droneReal,droneSim)
