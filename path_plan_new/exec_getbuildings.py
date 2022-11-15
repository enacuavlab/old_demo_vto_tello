#!/usr/bin/python3

import struct
import time
import socket
from threading import Thread

import sys
import argparse
import csv
import math
import copy
import numpy as np

#--------------------------------------------------------------------------------
#
# ./exec_getbuildings.py -i material/'Take 2022-11-08 08.26.58 AM.csv' -o material/outputfromtake.csv
# or
# ./exec_getbuildings.py -o material/outputfromnatnet.csv
#
#--------------------------------------------------------------------------------

# This program gets top surface as building rigidbody (Building_xxx)

# Option 1: From Optitrack Motive tracking data as a csv extracted file
#   Export extracting data
#     Axis Convention : Custom
#       X Axis : Left(X+)
#       Y Axis : Backward(Z-)
#       Z Axis : Up(Y+)

# Option 2: From Optitrack Natnet markers live positions 
#   This programs uses MoCapData.py from natnet SDK 4.0 (windows package)
#   This program gets data from Motive Server v3, with following settings:
#   Local Interface : 192.168.1.231
#   Transmission Type : Multicast
#   Labeled Markers : OFF
#   Unlabeled Markers : OFF
#
#   Asset Markers : ON
#   Rigid Bodies : ON or OFF
#
#   Up Axis : Z-Axis
#
#   Command Port : 1510
#   Data Port : 1511
#   Multicast Interface : 239.255.42.99


#--------------------------------------------------------------------------------
class LightBuilding():
  def __init__(self,name,vertices): # Buildings(obstacles) are defined by coordinates of their vertices.
    self.name = name
    self.vertices = vertices

#--------------------------------------------------------------------------------
def sort_verts(buildings,buildingList):
  for item1 in buildings.items():
    pts = np.empty((len(item1[1].items()),2))
    for i,item2 in enumerate(item1[1].items()): pts[i] = np.array(item2[1][0:2])
    angles = []
    (x0,y0)=(0,0)
    for i in pts: (x0,y0) = (i[0]+x0,i[1]+y0)
    (x0,y0) = (x0/len(pts),y0/len(pts))
    for i in pts:
      (dx,dy) = (i[0]-x0,i[1]-y0)
      angles.append(math.degrees(math.atan2(dy,dx)))
    tmp = [copy.copy(x) for y,x in sorted(zip(angles,pts), key=lambda pair: pair[0])]
    for i,item in enumerate(tmp):  pts[i] = item
    verts = np.empty((len(item1[1].items()),3))
    for i,item2 in enumerate(item1[1].items()): verts[i] = (pts[i][0],pts[i][1],4.2)
    b = LightBuilding(item1[0],verts)
    buildingList.append(b)

  
def csvfile_parse(inputfile,buildingList):
  with open(inputfile, newline='') as csvfile:
    csvreader = csv.reader(csvfile)
    for j in range(3): dummy = next(csvreader)
    fields = next(csvreader)
    for j in range(3): dummy = next(csvreader)
    values = next(csvreader)
    buildings = {}
    for i in range(2,len(fields),3):
      buildingName,markerName  = fields[i].split(':')
      floatLst=[float(x)/1000.0 for x in values[i:i+3]]
      if (not buildings) or (not buildingName in buildings): buildings[buildingName]={markerName:floatLst}
      buildings[buildingName].update({markerName:floatLst})
    sort_verts(buildings,buildingList)


#--------------------------------------------------------------------------------
Vector3 = struct.Struct( '<fff' )

def natnet_parse(in_socket,buildings):
  data=bytearray(0)
  # 64k buffer size
  recv_buffer_size=64*1024
  data, addr = in_socket.recvfrom( recv_buffer_size )
  if len( data ) > 0 :
    message_id = int.from_bytes( data[0:2], byteorder='little' )
    packet_size = int.from_bytes( data[2:4], byteorder='little' )
    if message_id == 7 : # NAT_FRAMEOFDATA :
      offset = 4
      frame_number = int.from_bytes( data[offset:offset+4], byteorder='little' )
      offset += 4
      marker_set_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
      offset += 4
      for i in range( 0, marker_set_count ):
        model_name, separator, remainder = bytes(data[offset:]).partition( b'\0' )
        offset += len( model_name ) + 1
        buildingName = model_name.decode( 'utf-8' )
        #print("Model name: ",buildingName)
        marker_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
        offset += 4
        #print( "Marker Count    : ", marker_count )
        for j in range( 0, marker_count ):
          floatLst = Vector3.unpack( data[offset:offset+12] )
          offset += 12
          #print( "\tMarker %3.1d : [%3.2f,%3.2f,%3.2f]"%( j, pos[0], pos[1], pos[2] ))
          if buildingName.startswith('Building_'):
            markerName = str(j)
            if (not buildings) or (not buildingName in buildings): buildings[buildingName]={markerName:floatLst}
            buildings[buildingName].update({markerName:floatLst})


def natnet_get(buildingList):
  data_sock = socket.socket( socket.AF_INET,socket.SOCK_DGRAM,0)
  data_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  data_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton("239.255.42.99") + socket.inet_aton("0.0.0.0"))
  try:
    data_sock.bind( ("0.0.0.0", 1511) )
  except socket.error as msg:
    print("ERROR: data socket error occurred:\n%s" %msg)
    print("  Check Motive/Server mode requested mode agreement.  You requested Multicast ")
  stop_threads = False
  buildings = {}
  data_thread = Thread( target = natnet_parse, args = (data_sock,buildings ))
  data_thread.start()
  time.sleep(3)
  stop_threads = True
  data_thread.join()
  sort_verts(buildings,buildingList)


#--------------------------------------------------------------------------------
if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument( '-i', '--input_csvtakefile')
  parser.add_argument( '-o', '--output_csvfile')
  args = parser.parse_args()

  if args.output_csvfile:
    buildingList = []
    if args.input_csvtakefile:
      print("Building from take csv file")
      csvfile_parse(args.input_csvtakefile,buildingList)
    else:
      print("Searching for live multicast buildings")
      natnet_get(buildingList)

    with open(args.output_csvfile, 'w') as out:
      for elt in buildingList:
        stringvalues = elt.name
        for vert in elt.vertices:
          stringvalues = stringvalues + ("[%.8f %.8f %.8f]" % (vert[0],vert[1],vert[2]))
        out.write(stringvalues+'\n')
