#!/usr/bin/python3

import matplotlib.pyplot as plt

#--------------------------------------------------------------------------------
class Drawing():
  def __init__(self):

    self.fig, self.gs0 = plt.subplots()
    self.gs0.clear()
    self.fig.subplots_adjust(left=0.25, bottom=0.25)
    self.gs0.set_xlabel('Time [s]')
    self.gs0.set_xlim(-5, 5)
    self.gs0.set_ylim(-5, 5)
    self.gs0.grid()
  
    self.gs0.plot(1.3,1.6,color='green',marker='o',markersize=12)


  def refresh(self,updatedlst):
    for elt in updatedlst:
      print(elt)
#    self.gs0.plot(x,y,color='green',marker='o',markersize=12)
#    self.fig.clear()
    self.fig.canvas.draw_idle()


  def start(self):
    plt.show()
