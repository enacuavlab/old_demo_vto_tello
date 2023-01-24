#!/usr/bin/python3

from natnet4 import Rigidbody,Thread_natnet
from mission import Thread_mission
from command import Thread_command
from vehicle import Vehicle
from netdrone import initNetDrone
from drawing import Drawing

import argparse
import time

import threading,queue
import numpy as np

#------------------------------------------------------------------------------
acTarg = [888,'Helmet']

optiFreq = 20 # Check that optitrack stream at least with this value

#------------------------------------------------------------------------------
class Flag(threading.Event):
  def __bool__(self):
    return self.is_set()

#------------------------------------------------------------------------------
def getData(flag,com):
  xval = 0.0
  yval = 0.0
  while not flag:
    xval = xval + np.random.random()/10.0
    yval = yval + np.random.random()/10.0
    com.put([xval,yval])
  #  print(xval,yval)
    time.sleep(1)


def displayData(flag,com,drawing):
  while not flag:
    if not com.empty():
      (x,y) = com.get()
      print(x, y)
      drawing.refresh(x,y)


#------------------------------------------------------------------------------
def main(droneReal,droneSim):

  vehicleListReal = []
  rigidBodyDict = {}
  rigidBodyDict[acTarg[0]] = Rigidbody(acTarg[0])
  for ac in droneReal:
    vehicleListReal.append(Vehicle(ac))
    rigidBodyDict[ac]=Rigidbody(ac)

  vehicleListSim = []
  for ac in droneSim:
    vehicleListSim.append(Vehicle(ac))

  flag = Flag()

  if vehicleListReal:
    try:
      threadMotion = Thread_natnet(flag,rigidBodyDict,optiFreq)
      threadMotion.start()
    except ValueError as msg:
      print(msg)
      exit()

  commands = queue.Queue()

  drawing = Drawing()

  threadMission = Thread_mission(drawing,flag,rigidBodyDict,acTarg[0])
  threadMission.start()

  threadCommand = Thread_command(flag,commands)
  threadCommand.start()


#  proc1 = threading.Thread(target=getData,args=(flag,commands,))
#  proc1.start()
#
#  proc2 = threading.Thread(target=displayData,args=(flag,commands,drawing))
#  proc2.start()

  try:
    drawing.start()

  except KeyboardInterrupt:
    print("KeyboardInterrupt")
    flag.set()

  finally:
    print("finally")
    flag.set()
#    proc1.join()
#    proc2.join()
    if vehicleListReal: threadMotion.join()
    threadMission.join()
    threadCommand.join()


#------------------------------------------------------------------------------
def toString(param):
  return((param[0],param[1]))

#------------------------------------------------------------------------------
if __name__=="__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--ar', nargs='+', dest='realacs', type=int)
  parser.add_argument('--as', nargs='+', dest='simuacs', type=int)
  parser.add_argument('--v0', nargs='+', dest='toto', type=toString)
  args = parser.parse_args()

  print(args.toto)

  ret=True
  droneReal = {}
  droneSim = {}
  if args.realacs is not None:
    ret = False
    ret,droneAddrs = initNetDrone(args.realacs)
    if ret: droneReal = droneAddrs

  if args.simuacs is not None:
    for elt in args.simuacs: droneSim[elt]=elt

  if ret:
    main(droneReal,droneSim)
