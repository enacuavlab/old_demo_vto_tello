#!/usr/bin/python3

import threading
import queue
import time
import threading

import numpy as np

from vehicle import Vehicle

#------------------------------------------------------------------------------
telloFreq = 10

#------------------------------------------------------------------------------
def compute_flow(vehicles):

  flow_vels = np.zeros([len(vehicles),3])

  V_sink    = np.zeros([len(vehicles),2]) # Velocity induced by sink element
  V_source  = np.zeros([len(vehicles),2]) # Velocity induced by source elements
  V_sum     = np.zeros([len(vehicles),2]) # V_gamma + V_sink + V_source
  V_normal  = np.zeros([len(vehicles),2]) # Normalized velocity
  V_flow    = np.zeros([len(vehicles),2]) # Normalized velocity inversly proportional to magnitude
  V_norm    = np.zeros([len(vehicles),1]) # L2 norm of velocity vector

  W_flow    = np.zeros([len(vehicles),1]) # Vertical velocity component (to be used in 3-D scenarios)

  for f,vehicle in enumerate(vehicles):

    # Cartesian velocity reprsentation by 2D sink
    # (Velocity induced by 2D point sink, eqn. 10.2 & 10.3 in Katz & Plotkin:)
    V_sink[f,0] = (-vehicle.sink_strength*(vehicle.position[0]-vehicle.goal[0]))/\
                  (2*np.pi*((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2))
    V_sink[f,1] = (-vehicle.sink_strength*(vehicle.position[1]-vehicle.goal[1]))/\
                  (2*np.pi*((vehicle.position[0]-vehicle.goal[0])**2+(vehicle.position[1]-vehicle.goal[1])**2))

    othervehicleslist = vehicles[:f] + vehicles[f+1:]
    for othervehicle in othervehicleslist:
      # Cartesian velocity reprsentation by 2D source
      V_source[f,0] += (othervehicle.source_strength*(vehicle.position[0]-othervehicle.position[0]))/\
                       (2*np.pi*((vehicle.position[0]-othervehicle.position[0])**2+(vehicle.position[1]-othervehicle.position[1])**2))
      V_source[f,1] += (othervehicle.source_strength*(vehicle.position[1]-othervehicle.position[1]))/\
                       (2*np.pi*((vehicle.position[0]-othervehicle.position[0])**2+(vehicle.position[1]-othervehicle.position[1])**2))

    # Total velocity induced :
    V_sum[f,0] = V_sink[f,0] + V_source[f,0]
    V_sum[f,1] = V_sink[f,1] + V_source[f,1]

    # L2 norm of flow velocity:
    V_norm[f] = (V_sum[f,0]**2 + V_sum[f,1]**2)**0.5
    # Normalized flow velocity:
    V_normal[f,0] = V_sum[f,0]/V_norm[f]
    V_normal[f,1] = V_sum[f,1]/V_norm[f]

    # Flow velocity inversely proportional to velocity magnitude:
    V_flow[f,0] = V_normal[f,0]/V_norm[f]
    V_flow[f,1] = V_normal[f,1]/V_norm[f]

    flow_vels[f,:] = [V_flow[f,0],V_flow[f,1],W_flow[f,0]]

  return flow_vels

#------------------------------------------------------------------------------
class Thread_mission(threading.Thread):

  def __init__(self,quitflag,commands,vehicles,threadsim):
    threading.Thread.__init__(self)
    self.quitflag = quitflag
    self.commands = commands
    self.vehicles = vehicles
    self.threadsim = threadsim
    self.suspend = True
    self.telloPeriod = 1/telloFreq



  def run(self):

    while (self.suspend): time.sleep(self.telloPeriod)

    print("runnnig MISSION")

#    self.commands.put(('command',))
#    self.commands.put(('streamon',))
#    time.sleep(1)
#    self.commands.put(('takeoff',))
#    time.sleep(7)
    self.guidanceLoop() # drone should be flying to have position from optitrack
    self.commands.put(('land',))


  def guidanceLoop(self):
    unvalidcpt = 0
    loop_incr = 0

    flyings=[]
    for elt in self.vehicles:
      if (elt!=888): flyings.append(Vehicle(elt))

    print("looping MISSION")
    try: 

      while not self.quitflag and loop_incr < 15000 and not (self.suspend):
        loop_incr = loop_incr + 1
        time.sleep(self.telloPeriod)

        if (self.vehicles[888][0]):               # check suspended position capture for real target 
          (pos,val,vel,head)=self.vehicles[888][2](888)
          if not val:
            unvalidcpt = unvalidcpt+1
            if unvalidcpt == 10: break
            else: continue
          else: unvalidcpt= 0

        (targetPos,val,vel,head)=self.vehicles[888][2](888)
        for elt in self.vehicles:
          if (elt != 888):
            (pos,val,vel,head)=self.vehicles[elt][2](elt)
            for vel in flyings:
              if (vel.ID == elt):
                vel.update(pos,vel,head,targetPos,5)

#        flow_vels = compute_flow(flyings)
#  
#        for i,v in enumerate(flyings):
#          (cmd,cmd_val)=v.apply_flow(flow_vels[i])
#          if (self.vehicles[v.ID][0]): self.commands.put((cmd,v.ID))
#          else: self.threadsim.put(v.ID,cmd_val)

 
    finally: 
      print("Thread_mission stop")


  def trigger(self):
    if self.suspend: self.suspend = False
    else: self.suspend = True
