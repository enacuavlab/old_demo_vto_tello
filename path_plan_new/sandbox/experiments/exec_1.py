#!/usr/bin/python3

from natnet4 import Rigidbody,Thread_natnet
from mission import Thread_mission
from command import Thread_command
from vehicle import Vehicle
from netdrone import initNetDrone

import argparse
import queue
import time
import threading

import matplotlib.pyplot as plt

#------------------------------------------------------------------------------
#tellos_selected = (65,)

acTarg = [888,'Helmet']

optiFreq = 20 # Check that optitrack stream at least with this value

#------------------------------------------------------------------------------
class Flag(threading.Event):
  def __bool__(self):
    return self.is_set()

#------------------------------------------------------------------------------
def main(droneAddrs):

  vehicleList = [];
  rigidBodyDict = {};
  rigidBodyDict[acTarg[0]] = Rigidbody(acTarg[0])
  for ac in droneAddrs:
    vehicleList.append(Vehicle(ac))
    rigidBodyDict[ac]=Rigidbody(ac)

  flag = Flag()

  threadMotion = Thread_natnet(flag,rigidBodyDict,optiFreq)
  threadMotion.start()

  commands = queue.Queue()

  fig, gs0 = plt.subplots()
  gs0.clear()
  fig.subplots_adjust(left=0.25, bottom=0.25)
  gs0.set_xlabel('Time [s]')
  gs0.set_xlim(-5, 5)
  gs0.set_ylim(-5, 5)
  gs0.grid()

  threadMission = Thread_mission(gs0,fig,flag,rigidBodyDict,acTarg[0])
  threadMission.start()

  threadCommand = Thread_command(flag,commands)
  threadCommand.start()

  try:
    plt.show()

  except KeyboardInterrupt:
    flag.set()

  finally:
    flag.set()
    threadMission.join()
    threadMotion.join()
    threadCommand.join()


#------------------------------------------------------------------------------
if __name__=="__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--ac', nargs='+', type=int)
  args = parser.parse_args()

  for _, selected in parser.parse_args()._get_kwargs():
    if selected is not None:
      ret,droneAddrs = initNetDrone(selected)
      if ret:
        main(droneAddrs)
