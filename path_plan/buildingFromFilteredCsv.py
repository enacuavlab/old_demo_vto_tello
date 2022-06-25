#!/usr/bin/python3
import matplotlib.pyplot as plt
import csv
import math
import copy
import numpy as np

filename = "testFiltered.csv"

buildings = {}

def clocked(pts):
  angles = []
  (x0,y0)=(0,0)
  for i in pts: (x0,y0) = (i[0]+x0,i[1]+y0)
  (x0,y0) = (x0/len(pts),y0/len(pts))
  for i in pts: 
    (dx,dy) = (i[0]-x0,i[1]-y0)
    angles.append(math.degrees(math.atan2(dy,dx)))
  tmp = [copy.copy(x) for y,x in sorted(zip(angles,pts), key=lambda pair: pair[0])]
  for i,item in enumerate(tmp):  pts[i] = item


if __name__ == '__main__':
  plt.xlim(-5, 5)
  plt.ylim(-5, 5)
  plt.grid()
  with open(filename, newline='') as csvfile:
    csvreader = csv.reader(csvfile)
    fields = next(csvreader)
    values = next(csvreader)
    for i in range(0,len(fields),3):
      buildingName,markerName  = fields[i].split(':')
      floatLst=[float(x)/1000.0 for x in values[i:i+3]]
      if (not buildings) or (not buildingName in buildings): buildings[buildingName]={markerName:floatLst}
      buildings[buildingName].update({markerName:floatLst})

    for item1 in buildings.items(): 
      pts = np.empty((len(item1[1].items()),2))
      for i,item2 in enumerate(item1[1].items()):
        pts[i] = np.array(item2[1][0:2])
      clocked(pts)
      if(item1[0] == 'Building_884'): 
        for i,elt in enumerate(pts[:-1]): plt.plot([pts[i][0],pts[i+1][0]],[pts[i][1],pts[i+1][1]],color='red')
        plt.plot([pts[0][0],pts[len(pts)-1][0]],[pts[0][1],pts[len(pts)-1][1]],color='red')


  plt.show()
