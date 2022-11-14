#!/usr/bin/python3

import struct
import time
import socket
from threading import Thread

import json
import argparse
import numpy as np


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

# From Optitrack Natnet rigidbodies live positions
#   This programs inspired  from MoCapData.py natnet SDK 4.0 (windows package)
#   This program gets data from Motive Server v3, with following settings:
#   Local Interface : 192.168.1.231
#   Transmission Type : Multicast
#   Labeled Markers : On
#   Unlabeled Markers : On
#   Asset Markers : On
#   Rigid Bodies : On
#   ...
#   Command Port : 1510
#   Data Port : 1511
#   Multicast Interface : 239.255.42.99

#--------------------------------------------------------------------------------
#  tello_address = ('192.168.10.61', 8889)
#  sock.sendto('command'.encode(encoding="utf-8"),tello_address)
#  sock.sendto('takeoff'.encode(encoding="utf-8"),tello_address)
#  sock.sendto('land'.encode(encoding="utf-8"),tello_address)
#  cmd=("rc %d %d %d %d"%(roll,pitch,throttle,yaw))
#  sock.sendto(''.encode(encoding="utf-8"),tello_address)

#--------------------------------------------------------------------------------
#acDict = {60:[('TELLO-ED4310')],65:[('TELLO-F0B594')]}
#acDict = {60:[('TELLO-ED4310')]}
#acDict = {65:[('TELLO-F0B594')]}
#acDict = {66:[('TELLO-99CE21')]}
#acTarg = [888,'Helmet']

#optiFreq = 20
#telloFreq = 10
#telloSpeed = 0.3

#sourceStrength = 0.95 # Tello repelance

#--------------------------------------------------------------------------------
Vector3 = struct.Struct( '<fff' )

def natnet_parse(in_socket,buildings):
  data=bytearray(0)
  # 64k buffer size
  recv_buffer_size=64*1024
  data, addr = in_socket.recvfrom( recv_buffer_size )


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
#    for val0, val1, val2, val3, val4, val5 in retmat.values():
#      b = Building(val0,np.array(val1))
#      pts = b.vertices
#      b.vertices = np.array(val1) # udpate vertices
#      b.pcp = np.array(val2)
#      b.pb = np.array(val3)
#      b.nop = val4
#      b.K_inv = np.array(val5)
#      buildingList.append(b)

    natnet_get(buildingList)
