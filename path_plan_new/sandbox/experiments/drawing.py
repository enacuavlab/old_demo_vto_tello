#!/usr/bin/python3

import matplotlib.pyplot as plt
import matplotlib.animation as animation


#--------------------------------------------------------------------------------
class Drawing():
  def __init__(self,vehiclelst):

    self.vehiclelst = vehiclelst
    self.fig, self.ax = plt.subplots()
    self.fig.subplots_adjust(left=0.25, bottom=0.25)
    self.ax.set_xlabel('Time [s]')
    self.ax = plt.axes(xlim=(-5,5),ylim=(-5,5))
    self.ax.grid()


  def update(self,i,scat):
  
    data = np.array([[self.vehiclelst[0].position[0],self.vehiclelst[0].position[1],0,0]])
    for i,elt in enumerate(self.vehiclelst, start=1): data = np.append(data,np.array([[elt.position[0],elt.position[1],0,0]]),axis=0)
  
    scat.set_offsets(data[:, :2])                     # x and y
    scat.set_sizes(300 * abs(data[:, 2])**1.5 + 100)  # size
    scat.set_array(data[:, 3])                        # color
  
    return scat,


  def start(self):
    ani = animation.FuncAnimation(self.fig,self.update,fargs=(self.ax.scatter([],[])),interval=1,blit=True)
    plt.show()
