#!/usr/bin/python3

import threading,queue,time,argparse
import numpy as np

from natnet4 import Rigidbody,Thread_natnet
from mission import Thread_mission
from command import Thread_commandReal
from simulation import Thread_commandSim, SimBody
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


  if (len(bodies)<len(vehicles)):
    threadCmdSim = Thread_commandSim(flag,vehicles)
    threadCmdSim.start()
  else: threadCmdSim =  None

  if (len(vehicles)>1):
    threadMission = Thread_mission(flag,commands,vehicles,threadCmdSim)     # for flying and simulated tellos
    threadMission.start()
  else: threadMission =  None


  if (len(bodies)>0) and (len(vehicles)>1):
    threadCmdReal = Thread_commandReal(flag,commands,vehicles) # for flying tellos
#    threadCmdReal.start()
  else: 
    threadCmdReal = None


  try:

    DrawingGL(vehicles,threadCmdSim,threadMission).start()

  except KeyboardInterrupt:
    print("KeyboardInterrupt")
    flag.set()

  finally:
    print("finally")
    flag.set()
#    if (len(bodies)<len(vehicles)): threadCmdSim.join()
#    if (len(bodies)>0) and (len(vehicles)>1): threadCmdReal.join()
#    if (len(vehicles)>1): threadMission.join()


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
    simus = {}

    if not args.targSim:                             # first element is the simulated or real target
      bodies[acTarg[0]] = Rigidbody(acTarg[0])
      vel = Vehicle(acTarg[0])
      vehicles[acTarg[0]]=(True,vel,(lambda arg: (bodies[arg].position,bodies[arg].valid,bodies[arg].velocity,bodies[arg].heading)))
    else:
      simus[acTarg[0]] = SimBody(acTarg[0])
      vel = Vehicle(args.targSim)
      vel.position = (4.0,0.0,3.0)
      simus[acTarg[0]] = SimBody(acTarg[0])
      simus[acTarg[0]].position = vel.position
      vehicles[acTarg[0]]=(False,vel,(lambda arg: (simus[arg].position,simus[arg].valid,simus[arg].velocity,simus[arg].heading)),(lambda arg,pos:(self.simus[arg].position:=pos)))

    for elt in droneReal:
      bodies[elt] = Rigidbody(elt)
      vel = Vehicle(elt)  
      vehicles[elt]=(True,vel,(lambda arg: (bodies[arg].position,bodies[arg].valid,bodies[arg].velocity,bodies[arg].heading)),droneReal[elt][1])

    for elt in droneSim:
      vel = Vehicle(elt)
      vel.position = droneSim[elt] 
      simus[elt] = SimBody(elt)
      simus[elt].position = vel.position
      vehicles[elt]=(False,vel,(lambda arg: (simus[arg].position,simus[arg].valid,simus[arg].velocity,simus[arg].heading)))

    main(bodies,vehicles)
