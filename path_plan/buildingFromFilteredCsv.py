#!/usr/bin/python3

import csv

filename = "testFiltered.csv"

buildings = {}
markers = {}

if __name__ == '__main__':
  with open(filename, newline='') as csvfile:
#   csvreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    csvreader = csv.reader(csvfile)
    fields = next(csvreader)
    values = next(csvreader)
    #for row in csvreader: row.append(row)
    for index in range(0,len(fields),3):
      buildingName,markerName  = fields[index].split(':')
      if building is {}: 
      print(buildingName)

#      markers[fields[index]]=(values[index:index+2])
#   for elt in markers: print(elt)  
#    print(fields)
#    print(values)
    #print('Field names are:' + ', '.join(field for field in fields))
    #print('Field names are:' + ', '.join(values for value in values))
