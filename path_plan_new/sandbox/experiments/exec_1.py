#!/usr/bin/python3

import threading,queue,time,argparse
import numpy as np

from natnet4 import Rigidbody,Thread_natnet
from mission import Thread_mission
from command import Thread_commandReal
from simulation import Thread_commandSim
from vehicle import Vehicle
from netdrone import initNetDrone
from drawingGL import DrawingGL

"""
#------------------------------------------------------------------------------

# simulated target (ex:888)

./exec_1.py --ts 888
./exec_1.py --ts 888 --as 45[-1.1,-2.2,3.3]
./exec_1.py --ts 888 --ar 65
./exec_1.py --ts 888 --as 45[-1.1,-2.2,3.3] --ar 65

# real target (should be 888 from optitrack)

./exec_1.py
./exec_1.py --as 45[-1.1,-2.2,3.3]
./exec_1.py --ar 65
./exec_1.py --as 45[-1.1,-2.2,3.3] --ar 65

------------------------------------------------------------------------------
"""


acTarg = [888,'Helmet']

optiFreq = 20 # Check that optitrack stream at least with this value

FPS = 30

#------------------------------------------------------------------------------
class Flag(threading.Event):
  def __bool__(self):
    return self.is_set()

#------------------------------------------------------------------------------
def main(targSim,droneReal,droneSim):

  vehicleListReal = []
  vehicleListSim = []
  rigidBodyDict ={}
  if not targSim:
     rigidBodyDict[acTarg[0]] = Rigidbody(acTarg[0])
  else:
    vel = Vehicle(targSim)
    vel.position = (4.0,0.0,3.0)
    vehicleListSim.append(vel)

  for elt in droneSim:
    vel = Vehicle(elt)
    vel.position = droneSim[elt]
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

  threadMission = Thread_mission(flag,targSim,rigidBodyDict,acTarg[0])
  threadMission.start()

  threadCmdReal = Thread_commandReal(flag,commands)
  threadCmdReal.start()

  if vehicleListSim:
    threadCmdSim = Thread_commandSim(flag,targSim,vehicleListSim,rigidBodyDict,acTarg[0])
    threadCmdSim.start()

  try:
    if vehicleListSim:
      DrawingGL(FPS,vehicleListSim,rigidBodyDict,threadCmdSim.triggersim).start()
    else:
      DrawingGL(FPS,vehicleListSim,rigidBodyDict,None).start()


  except KeyboardInterrupt:
    print("KeyboardInterrupt")
    flag.set()

  finally:
    print("finally")
    flag.set()
    if vehicleListSim: threadCmdSim.join()
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
  parser.add_argument('--ts', dest='targSim', type=int)
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
    main(args.targSim,droneReal,droneSim)
