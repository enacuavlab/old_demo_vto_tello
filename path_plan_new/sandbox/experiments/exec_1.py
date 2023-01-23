#!/usr/bin/python3

from natnet4 import Rigidbody,Thread_natnet
from mission import Thread_mission
from command import Thread_command
from vehicle import Vehicle
from netdrone import initNetDrone

import argparse
import time

import matplotlib.pyplot as plt

import threading,queue
from matplotlib.animation import FuncAnimation
import numpy as np

#------------------------------------------------------------------------------
#tellos_selected = (65,)

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


def displayData(flag,com,fig,gs0):
  while not flag:
    if not com.empty():
      (x,y) = com.get()
      print(x, y)
      gs0.plot(x,y,color='green',marker='o',markersize=12)
      fig.canvas.draw_idle()


#------------------------------------------------------------------------------
def main(droneAddrs):

#  vehicleList = [];
#  rigidBodyDict = {};
#  rigidBodyDict[acTarg[0]] = Rigidbody(acTarg[0])
#  for ac in droneAddrs:
#    vehicleList.append(Vehicle(ac))
#    rigidBodyDict[ac]=Rigidbody(ac)
#
  flag = Flag()
#
#  threadMotion = Thread_natnet(flag,rigidBodyDict,optiFreq)
#  threadMotion.start()
#
  commands = queue.Queue()
#
  fig, gs0 = plt.subplots()
  gs0.clear()
  fig.subplots_adjust(left=0.25, bottom=0.25)
  gs0.set_xlabel('Time [s]')
  gs0.set_xlim(-5, 5)
  gs0.set_ylim(-5, 5)
  gs0.grid()

  gs0.plot(1.3,1.6,color='green',marker='o',markersize=12)

#  threadMission = Thread_mission(gs0,fig,flag,rigidBodyDict,acTarg[0])
#  threadMission.start()
#
#  threadCommand = Thread_command(flag,commands)
#  threadCommand.start()

#  try:
#    plt.show()

#  except KeyboardInterrupt:
#    flag.set()

#  finally:
#    flag.set()
#    threadMission.join()
#    threadMotion.join()
#    threadCommand.join()


  proc1 = threading.Thread(target=getData,args=(flag,commands,))
  proc1.start()

  proc2 = threading.Thread(target=displayData,args=(flag,commands,fig,gs0))
  proc2.start()

  try:
    plt.show()

  except KeyboardInterrupt:
    print("KeyboardInterrupt")
    flag.set()

  finally:
    print("finally")
    flag.set()
    proc1.join()
    proc2.join()



#------------------------------------------------------------------------------
if __name__=="__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--ac', nargs='+', type=int)
  args = parser.parse_args()

  main([])
#  for _, selected in parser.parse_args()._get_kwargs():
#    if selected is not None:
#      ret,droneAddrs = initNetDrone(selected)
#      if ret:
#        main(droneAddrs)
