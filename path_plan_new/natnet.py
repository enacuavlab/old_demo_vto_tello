#!/usr/bin/python3

import struct
import socket
import select
import threading



Vector3 = struct.Struct( '<fff' )
Quaternion = struct.Struct( '<ffff' )
FloatValue = struct.Struct( '<f' )

#------------------------------------------------------------------------------
class Thread_natnet(threading.Thread):

  def __init__(self,rigidbodydic):
    threading.Thread.__init__(self)
    self.running = True
    self.rigidbodydic = rigidbodydic
    self.data_sock = socket.socket( socket.AF_INET,socket.SOCK_DGRAM,0)
    self.data_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.data_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, socket.inet_aton("239.255.42.99") + socket.inet_aton("0.0.0.0"))
    try:
      self.data_sock.bind( ("0.0.0.0", 1511) )
    except socket.error as msg:
      print("ERROR: data socket error occurred:\n%s" %msg)
      print("  Check Motive/Server mode requested mode agreement.  You requested Multicast ")


  def run(self):
    data=bytearray(0)
    # 64k buffer size
    recv_buffer_size=64*1024
    while(self.running):
      data, addr = self.data_sock.recvfrom( recv_buffer_size )
      if len( data ) > 0 :
        message_id = int.from_bytes( data[0:2], byteorder='little' )
        packet_size = int.from_bytes( data[2:4], byteorder='little' )
        if message_id == 7 : # NAT_FRAMEOFDATA :
          offset = 4
          offset += 4
          marker_set_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
          offset += 4
    
          for i in range( 0, marker_set_count ): #  if Asset Markers : ON
            model_name, separator, remainder = bytes(data[offset:]).partition( b'\0' )
            offset += len( model_name ) + 1
            marker_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            for j in range( 0, marker_count ):
              floatLst = Vector3.unpack( data[offset:offset+12] )
              offset += 12
    
          offset += 4
          rigid_body_count = int.from_bytes( data[offset:offset+4], byteorder='little' )
          offset += 4
          for i in range( 0, rigid_body_count ): # if Rigid Bodies : ON
            new_id = int.from_bytes( data[offset:offset+4], byteorder='little' )
            offset += 4
            pos = Vector3.unpack( data[offset:offset+12] )
            offset += 12
            rot = Quaternion.unpack( data[offset:offset+16] )
            offset += 16
            marker_error, = FloatValue.unpack( data[offset:offset+4] )
            offset += 4
            param, = struct.unpack( 'h', data[offset:offset+2] )
            tracking_valid = ( param & 0x01 ) != 0
            offset += 2
            if tracking_valid:
              idName = str(new_id)
              if idName in self.rigidbodydic:
                self.rigidbodydic.update({idName:(pos,rot)})
