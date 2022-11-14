#!/usr/bin/python3

import json
import argparse
import numpy as np
from common import Building


#--------------------------------------------------------------------------------
#
# ./exec_run.py -i outputfromtake.json
# or 
# ./exec_run.py -i outputfromnatnet.json
#
#--------------------------------------------------------------------------------

# This program compute and send in realtime flight guidance to vehicles 
# - to track a goal
# - to avoid buildings
# - to avoid vehicles

#--------------------------------------------------------------------------------
if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument( '-i', '--input_jsonmatrix')
  args = parser.parse_args()

  if (args.input_jsonmatrix):
    retmat = {}
    with open(args.input_jsonmatrix, "r") as infile: retmat = json.load(infile)
    infile.close()

    buildingList = []
    for val0, val1, val2, val3, val4, val5 in retmat.values():
      b = Building(val0,np.array(val1))
      pts = b.vertices
      b.vertices = np.array(val1) # udpate vertices
      b.pcp = np.array(val2)
      b.pb = np.array(val3)
      b.nop = val4
      b.K_inv = np.array(val5)
      buildingList.append(b)
