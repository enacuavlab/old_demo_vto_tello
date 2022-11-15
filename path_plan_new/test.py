#!/usr/bin/python3

from natnet import Thread_natnet

import queue
import time

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
