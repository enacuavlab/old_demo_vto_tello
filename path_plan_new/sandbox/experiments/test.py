#!/usr/bin/env python3


from PyQt5 import QtCore,QtWidgets
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import numpy as np
import sys
import time

class Visualizer(object):
    def __init__(self):
        self.traces = dict()
        self.app = QtWidgets.QApplication(sys.argv)
        self.w = gl.GLViewWidget()
        self.w.opts['distance'] = 40
        self.w.setWindowTitle('pyqtgraph example: GLLinePlotItem')
        self.w.setGeometry(0, 110, 1920, 1080)
        self.w.show()

        self.phase = 0
        self.lines = 50
        self.points = 1000
        self.y = np.linspace(-10, 10, self.lines)
        self.x = np.linspace(-10, 10, self.points)

        for i, line in enumerate(self.y):
            y = np.array([line] * self.points)
            d = np.sqrt(self.x ** 2 + y ** 2)
            sine = 10 * np.sin(d + self.phase)
            pts = np.vstack([self.x, y, sine]).transpose()
            self.traces[i] = gl.GLScatterPlotItem(
                pos=pts,
                color=pg.glColor((i, self.lines * 1.3)),
                size=(i + 1) / 10,
                pxMode=True
            )
#            self.traces[i] = gl.GLLinePlotItem(
#                pos=pts,
#                color=pg.glColor((i, self.lines * 1.3)),
#                width=(i + 1) / 10,
#                antialias=True
#            )
            self.w.addItem(self.traces[i])

    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtWidgets.QApplication.instance().exec_()

    def set_plotdata(self, name, points, color, width):
#        self.traces[name].setData(pos=points, color=color, width=width)
        self.traces[name].setData(pos=points, color=color, size=width)

    def update(self):
        stime = time.time()
        for i, line in enumerate(self.y):
            y = np.array([line] * self.points)

            amp = 10 / (i + 1)
            phase = self.phase * (i + 1) - 10
            freq = self.x * (i + 1) / 10

            sine = amp * np.sin(freq - phase)
            pts = np.vstack([self.x, y, sine]).transpose()

            print(pts,pg.glColor((i, self.lines * 1.3)))

            self.set_plotdata(
                name=i, points=pts,
                color=pg.glColor((i, self.lines * 1.3)),
                width=3
            )
            self.phase -= .0002

        print('{:.0f} FPS'.format(1 / (time.time() - stime)))

    def animation(self):
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(10)
        self.start()


# Start event loop.
if __name__ == '__main__':
    v = Visualizer()
    v.animation()

