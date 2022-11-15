#!/usr/bin/python3

import struct
import socket
import select
import threading
import time
import queue



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



#------------------------------------------------------------------------------
#class Thread_mission(threading.Thread):
#  def __init__(self,commands,rigidBodyDict,vehicles,arena):
#    threading.Thread.__init__(self)
#    self.commands = commands
#
#  def run(self):
#    target_pos = np.zeros(3)
#    for i in range(1000):
#      if(not self.rigidBodyDict[acTarg[0]].valid):
#        if self.running:time.sleep(1)
#      else: break
#    if(self.rigidBodyDict[acTarg[0]].valid):
#      print("start mission")

#------------------------------------------------------------------------------
def main():

  rigidbodydic = {'888':None,'65':None,'60':None}

  threadMotion = Thread_natnet(rigidbodydic)
  threadMotion.start()


#  threadMission = Thread_mission(commands,rigidBodyDict,vehicleList,arena)
#  threadMission.start()

  commands = queue.Queue()

  try:
    while True:
      while not commands.empty():
        vtupple=commands.get()
        if (len(vtupple)==2):
          inoutDict[vtupple[1]][0].sendto(vtupple[0].encode(encoding="utf-8"),(inoutDict[vtupple[1]][1],inoutDict[vtupple[1]][2]))
        else:
          for ac in acDict:
            inoutDict[ac][0].sendto(vtupple[0].encode(encoding="utf-8"),(inoutDict[ac][1],inoutDict[ac][2]))

      time.sleep(0.01)

  except KeyboardInterrupt:
    print("\nWe are interrupting the program\n")
    time.sleep(1)
    threadMotion.stop()
#    for ac in acDict: inoutDict[ac][0].close()
    print("mainloop stopped")


#------------------------------------------------------------------------------
if __name__=="__main__":
  main()
