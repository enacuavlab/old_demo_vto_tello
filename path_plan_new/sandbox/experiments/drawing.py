#!/usr/bin/python3

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

        background = fig.canvas.copy_from_bbox(ax.bbox)
                    fig.canvas.restore_region(background)
                                ax.draw_artist(points)
            fig.canvas.blit(ax.bbox)
            fig.canvas.draw()

#--------------------------------------------------------------------------------
class Drawing():
  def __init__(self):

    self.fig, self.gs0 = plt.subplots()
    self.fig.subplots_adjust(left=0.25, bottom=0.25)
    self.anim = FuncAnimation(fig, ud, frames=100, interval=100, blit=True)


    self.gs0.set_xlabel('Time [s]')
    self.gs0.set_xlim(-5, 5)
    self.gs0.set_ylim(-5, 5)
    self.gs0.grid()
  
  def refreshlst(self,lst):
    for elt in lst:
      self.gs0.plot(elt.position[0],elt.position[1],color='green',marker='o',markersize=12)
      print((elt.ID,elt.position))
    self.fig.canvas.draw_idle()

  def refreshdic(self,dic):
    for i,elt in dic.items():
      if i != 888:
        self.gs0.plot(elt.position[0],elt.position[1],color='green',marker='o',markersize=12)
        print(i,elt.position)
#    self.gs0.plot(x,y,color='green',marker='o',markersize=12)
#    self.fig.clear()
    self.fig.canvas.draw_idle()


  def start(self):
    plt.show()
