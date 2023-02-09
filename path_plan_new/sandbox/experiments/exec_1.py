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
# real target (should be 888 from optitrack)

# full simulation
./exec_1.py --ts 888
./exec_1.py --ts 888 --as 45[-1.1,-2.2,3.3]

# hybrid real and simulation
./exec_1.py --ts 888 --ar 65
./exec_1.py --ts 888 --as 45[-1.1,-2.2,3.3] --ar 65

# full real
./exec_1.py
./exec_1.py --ar 65

# hybrid real and simulation
./exec_1.py --as 45[-1.1,-2.2,3.3]
./exec_1.py --as 45[-1.1,-2.2,3.3] --ar 65

------------------------------------------------------------------------------
"""


acTarg = [888,'Helmet']

optiFreq = 20 # Check that optitrack stream at least with this value

#------------------------------------------------------------------------------
class Flag(threading.Event):
  def __bool__(self):
    return self.is_set()

#------------------------------------------------------------------------------
def main(bodies,vehicles):

  flag = Flag()

  if len(bodies):
    try:
      threadMotion = Thread_natnet(flag,bodies,optiFreq)
      threadMotion.start()
    except ValueError as msg:
      print(msg)
      exit()

  commands = queue.Queue()

  threadMission = Thread_mission(flag,commands,vehicles)
  threadMission.start()

  threadCmdReal = Thread_commandReal(flag,commands,vehicles)
  threadCmdReal.start()

  if (len(bodies)<len(vehicles)):
    threadCmdSim = Thread_commandSim(flag,vehicles)
    threadCmdSim.start()
  else: threadCmdSim =  None

  try:

    if (len(bodies)<len(vehicles)): DrawingGL(vehicles,threadCmdSim).start()

  except KeyboardInterrupt:
    print("KeyboardInterrupt")
    flag.set()

  finally:
    print("finally")
    flag.set()
    if (len(bodies)<len(vehicles)): threadCmdSim.join()
    if (len(bodies)>0): 
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

    vehicles = {}
    bodies = {}

    if not args.targSim:                             # first element is the simulated or real target
      bodies[acTarg[0]] = Rigidbody(acTarg[0])
      vel = Vehicle(acTarg[0])
      vehicles[acTarg[0]]=(True,vel,(lambda arg: (bodies[arg].position,bodies[arg].valid)))
    else:
      vel = Vehicle(args.targSim)
      vel.position = (4.0,0.0,3.0)
      vehicles[acTarg[0]]=(False,vel,(lambda arg: (vehicles[arg][1].position,True)))

    for elt in droneReal:
      bodies[elt] = Rigidbody(elt)
      vel = Vehicle(elt)                             # real = True, vehicle, get_position/valid, ip/port addr
      vehicles[elt]=(True,vel,(lambda arg: (bodies[arg].position,bodies[arg].valid)),droneReal[elt][1])

    for elt in droneSim:
      vel = Vehicle(elt)
      vel.position = droneSim[elt]                   # real = False, vehicle, get_position
      vehicles[elt]=(False,vel,(lambda arg: (vehicles[arg][1].position,True)))

    main(bodies,vehicles)
