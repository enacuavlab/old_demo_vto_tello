#!/usr/bin/python3

import json
import argparse
import numpy as np 
from common import Building
import matplotlib.pyplot as plt


#--------------------------------------------------------------------------------

# ./display.py -i outputfromtake.json
# or 
# ./display.py -i outputfromnatnet.json

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

    for val0, val1, val2, val3, val4, val5 in retmat.values():
      b = Building(val0,np.array(val1))
      pts = b.vertices
      b.vertices = np.array(val1) # udpate vertices
      display(val0,pts,b.vertices)

    plt.show()
