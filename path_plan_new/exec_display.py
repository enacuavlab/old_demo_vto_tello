#!/usr/bin/python3

from common import Flow_Velocity_Calculation
from buildingOut import BuildingOut
from vehicle import Vehicle

import numpy as np

import json
import argparse
import matplotlib.pyplot as plt


#--------------------------------------------------------------------------------
#
# ./exec_display.py -i material/outputfromtake.json
# or 
# ./exec_display.py -i material/outputfromnatnet.json
#
#--------------------------------------------------------------------------------

# This program compute and display one guidance to vehicles
# - to track a goal
# - to avoid buildings
# - to avoid vehicles

#------------------------------------------------------------------------------
def run(buildingList):
  acDict = {60:[('TELLO-ED4310')],65:[('TELLO-F0B594')]}
  sourceStrength = 0.95 # Tello repelance
  vehicleList = [];
  for ac in acDict: vehicleList.append(Vehicle(ac,sourceStrength))

  vehicleList[0].Go_to_Goal(1.4,0,0,0) # altitude,AoA,t_start,Vinf=0.5,0.5,1.5
  vehicleList[1].Go_to_Goal(1.4,0,0,0) # altitude,AoA,t_start,Vinf=0.5,0.5,1.5
#  targetPos = np.array([2.0,-4.0,2.0])
  targetPos = np.array([4.0,0.0,2.0])

  vehicleList[0].Set_Goal(targetPos,5,0.0)
  vehicleList[1].Set_Goal(targetPos,5,0.0)
  vehicleList[0].Set_Position([-3.0,4,0])
#  vehicleList[0].Set_Position([-4,4,0])
#  vehicleList[0].Set_Position([0,1,0])
  vehicleList[1].Set_Position([-4,-4,0])
  vehicleList[0].Set_Velocity([0,0,0])
  vehicleList[1].Set_Velocity([0,0,0])

  flow_vels = Flow_Velocity_Calculation(vehicleList,buildingList)

  plt.plot(targetPos[0],targetPos[1],color='green',marker='o',markersize=12)
  for i,v in enumerate(vehicleList):
    plt.plot(vehicleList[i].position[0],vehicleList[i].position[1],color='red',marker='o',markersize=12)
    vspeed=(flow_vels[i]/np.linalg.norm(flow_vels[i]))
    plt.arrow(vehicleList[i].position[0],vehicleList[i].position[1],vspeed[0],vspeed[1], fc="k", ec="k",head_width=0.05, head_length=0.1 )


#--------------------------------------------------------------------------------
def display(name,pts,ofs):
  plt.text(pts[0][0],0.7+pts[0][1],name,fontsize=12)
  for i,elt in enumerate(pts[:-1]):
    plt.plot([pts[i][0],pts[i+1][0]],[pts[i][1],pts[i+1][1]],color='blue')
    plt.plot([ofs[i][0],ofs[i+1][0]],[ofs[i][1],ofs[i+1][1]],color='red')
  plt.plot([pts[0][0],pts[len(pts)-1][0]],[pts[0][1],pts[len(pts)-1][1]],color='blue')
  plt.plot([ofs[0][0],ofs[len(ofs)-1][0]],[ofs[0][1],ofs[len(ofs)-1][1]],color='red')


#--------------------------------------------------------------------------------
if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument( '-i', '--input_jsonmatrix')
  args = parser.parse_args()

  if (args.input_jsonmatrix):
    print("Display matrix results")
    retmat = {}
    with open(args.input_jsonmatrix, "r") as infile: retmat = json.load(infile)
    infile.close()

    plt.xlim(-5, 5)
    plt.ylim(-5, 5)
    plt.grid()

    buildingList = []
    for val0, val1, val2, val3, val4, val5 in retmat.values():
      b = BuildingOut(val0,np.array(val1))
      pts = b.vertices
      b.vertices = np.array(val1) # udpate vertices
      display(val0,b.vertices,pts)

      b.pcp = np.array(val2)
      b.pb = np.array(val3)
      b.nop = val4
      b.K_inv = np.array(val5)
      buildingList.append(b)

    run(buildingList)
    plt.show()
