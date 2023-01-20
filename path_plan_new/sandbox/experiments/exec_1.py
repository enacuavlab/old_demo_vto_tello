#!/usr/bin/python3

from natnet4 import Rigidbody,Thread_natnet
from vehicle import Vehicle
from netdrone import initNetDrone

import argparse

#------------------------------------------------------------------------------
#tellos_selected = (65,)

acTarg = [888,'Helmet']

optiFreq = 20 # Check that optitrack stream at least with this value

#------------------------------------------------------------------------------
def main(droneAddrs):

  vehicleList = [];
  rigidBodyDict = {};
  rigidBodyDict[acTarg[0]] = Rigidbody(acTarg[0])
  for ac in droneAddrs:
    vehicleList.append(Vehicle(ac))
    rigidBodyDict[ac]=Rigidbody(ac)

  threadMotion = Thread_natnet(rigidBodyDict,optiFreq)
  threadMotion.start()

#------------------------------------------------------------------------------
if __name__=="__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--ac', nargs='+', type=int)
  args = parser.parse_args()

  for _, selected in parser.parse_args()._get_kwargs():
    if selected is not None:
      ret,droneAddrs = initNetDrone(selected)
      ret = True
      if ret:
        main(droneAddrs)
