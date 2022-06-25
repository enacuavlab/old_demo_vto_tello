#!/usr/bin/python3

import csv

filename = "testFiltered.csv"

buildings = {}

if __name__ == '__main__':
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
      print(item1[0])
      for item2 in item1[1].items(): 
        print(item2[0])
        print(item2[1])
