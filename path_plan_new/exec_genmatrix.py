#!/usr/bin/python3

import json
import argparse
import numpy as np
from buildingIn import BuildingIn


#--------------------------------------------------------------------------------
#
# ./exec_genmatrix.py -i material/outputfromtake.csv -o material/outputfromtake.json
# or
# ./exec_genmatrix.py -i material/outputfromnatnet.csv -o material/outputfromnatnet.json
#
#--------------------------------------------------------------------------------

# This program computes building avoidance matrix

#--------------------------------------------------------------------------------

def storenonaligned(pointDES, pointSRC, pointA, buildings, key1, key2):
    ang = np.math.degrees(np.math.atan2(pointA[1]-pointSRC[1], pointA[0]-pointSRC[0]) 
            - np.math.atan2(pointDES[1]-pointSRC[1], pointDES[0]-pointSRC[0]))
    if (ang < 0): ang = ang + 360
    if (ang < 177.0 or ang > 183.0):
      if (not buildings) or (not key1 in buildings): buildings[key1]={key2:pointSRC.tolist()}
      buildings[key1].update({key2:pointSRC.tolist()})


#--------------------------------------------------------------------------------
if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument( '-i', '--input_csvfile')
  parser.add_argument( '-o', '--output_jsonmatrix')
  args = parser.parse_args()

  buildings_ref = {}
  if (args.input_csvfile and args.output_jsonmatrix):
    print("Generate matrix from take or natnet csv file")
    with open(args.input_csvfile, newline='') as file:
      content = file.readlines()
      for row in content:
        parse = row.split('[')
        buildingName = parse[0]
        for i in range(1,len(parse)):
          values = parse[i][:-2].split()
          floatLst=[float(x)/1000.0 for x in values[0:3]]
          if (not buildings_ref) or (not buildingName in buildings_ref): buildings_ref[buildingName]={str(i):floatLst}
          buildings_ref[buildingName].update({str(i):floatLst})

  # Remove clockwize aligned points
  buildings = {}
  for key1 in buildings_ref.keys():
    for key2 in buildings_ref[key1].keys():
      elt = buildings_ref[key1][key2]
      if (key2 == '1'):
        a = np.array(elt[0:3])
        d = a
      elif (key2 == '2'):
        b = np.array(elt[0:3])
        e = b
      else:
        c = np.array(elt[0:3])
        storenonaligned(c,b,a,buildings,key1,key2)
        a = b
        b = c
    storenonaligned(d,b,a,buildings,key1,'2')
    storenonaligned(e,d,b,buildings,key1,'1')

  buildingList = []
  for itemA in buildings.items():
    verts = np.empty((len(itemA[1]),3),dtype=float)
    for i,itemB in enumerate(itemA[1].items()): 
      for j in range(3):verts[i][j] = itemB[1][j]*1000.0
    buildingList.append(BuildingIn(itemA[0],verts))

  data = {}
  for index,building in enumerate(buildingList):
    data[index] = (building.name,building.vertices.tolist(),building.pcp.tolist(),building.pb.tolist(),
                   building.nop,building.K_inv.tolist())
  with open(args.output_jsonmatrix, "w") as outfile: json.dump(data, outfile)
  outfile.close()
