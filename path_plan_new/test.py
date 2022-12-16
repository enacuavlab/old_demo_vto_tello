#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt


def plot1():
    nx, ny = .3, .3
    x = np.arange(-3, 3, nx)
    y = np.arange(-2, 2, ny)
    X, Y = np.meshgrid(x, y)

    plt.plot(X, Y, marker=".", color='k', linestyle='none')
    plt.xticks(x, rotation='45')
    plt.yticks(y)
    plt.title('grid plot with points, Single Colour')
    plt.show()

def plot2():
    nx, ny = .3, .3
    x = np.arange(-3, 3, nx)
    y = np.arange(-2, 2, ny)

    # rectangular grid with points
    X, Y = np.meshgrid(x, y)

    # derivative
    dy = X + np.sin(Y)
    dx = np.ones(dy.shape)

    # plot
    plt.quiver(X, Y, dx, dy, color='purple')
    plt.title('Direction Field for y\'(x) = x + sin(y)')
    plt.show()

def plot3():
    nx, ny = .3, .3
    x = np.arange(-3, 3, nx)
    y = np.arange(-2, 2, ny)
    X, Y = np.meshgrid(x, y)

    dy = X + np.sin(Y)
    dx = np.ones(dy.shape)

    # normalize arrows
    dyu = dy /np.sqrt(dx**2 + dy**2)
    dxu = dx /np.sqrt(dx**2 + dy**2)

    # plot
    plt.quiver(X, Y, dxu, dyu, color='purple')
    plt.title('Normalized Direction Field for y\'(x) = x + sin(y)')
    plt.show()

def plot4():
    nx, ny = .3, .3
    x = np.arange(-3, 3, nx)
    y = np.arange(-2, 2, ny)

    # rectangular grid with points
    X, Y = np.meshgrid(x, y)

    # derivative
    dy = X + np.sin(Y)
    dx = np.ones(dy.shape)

    # normalize arrows
    dyu = dy /np.sqrt(dx**2 + dy**2)
    dxu = dx /np.sqrt(dx**2 + dy**2)

    # plot
    plt.quiver(X, Y, dxu, dyu, color='teal', headaxislength=2, headlength=0,
               pivot='middle', scale=10, linewidth=.2, units='xy', width = .02, headwidth=1)
    plt.title('Normalized Direction Field for y\'(x) = x + sin(y) without arrows')
    plt.show()

def plot5():
    nx, ny = .5, .5
    x = np.arange(-3, 5, nx)
    y = np.arange(-3, 5, ny)
    X, Y = np.meshgrid(x, y)

    dy = X**2 - Y
    dx = np.ones(dy.shape)
    dyu = dy /np.sqrt(dx**2 + dy**2)
    dxu = dx /np.sqrt(dx**2 + dy**2)

    plt.quiver(X, Y, dxu, dyu, color='purple')
    plt.title('Normalized Direction Field for y\'(x) = x^2 - y')
    plt.show()

def plot6():
    nx, ny = .5, .5
    x = np.arange(-3, 5, nx)
    y = np.arange(-3, 5, ny)
    X, Y = np.meshgrid(x, y)

    dx = -X 
    dy = -Y 

    plt.quiver(X, Y, dx, dy, color='purple')
    plt.title('SINK')
    plt.show()

def plot7():
    nx, ny = .5, .5
    x = np.arange(-3, 5, nx)
    y = np.arange(-3, 5, ny)
    X, Y = np.meshgrid(x, y)

    dx = X 
    dy = Y 

    plt.quiver(X, Y, dx, dy, color='purple')
    plt.title('SOURCE')
    plt.show()


def plot8():
    nx, ny = .5, .5
    x = np.arange(-3, 5, nx)
    y = np.arange(-3, 5, ny)
    X, Y = np.meshgrid(x, y)

    dx = -2*X - 3*Y
    dy = 3*x - 2*Y

    plt.quiver(X, Y, dx, dy, color='purple')
    plt.title('SPIRAL')
    plt.show()

def plot9():
    dim = 20
    xarray = np.arange(-dim, dim)
    yarray = np.arange(-dim, dim)

    # (fluid) flow from a source at L to a sink at -L:
    L = dim/2
    x,y = np.meshgrid(xarray,yarray)
    vx = (x-L)/((x-L)**2+y**2) - (x+L)/((x+L)**2 +y**2)
    vy = y/((x-L)**2+y**2) - y/((x+L)**2 +y**2)

    # Masking the singularities at the poles:
    threshold = 0.33
    Mx = np.abs(vx) > threshold
    My = np.abs(vy) > threshold
    vx = np.ma.masked_array(vx, mask=Mx)
    vy = np.ma.masked_array(vy, mask=My)

    # plot the flow lines:
    plt.figure()
    plt.quiver(x,y, vx, vy, pivot='mid')
    plt.xlabel("$x$-axis")
    plt.ylabel("$y$-axis")
    plt.axis('equal')
    plt.show()


#plot1()
#plot2()
#plot3()
#plot4()
#plot5()
#plot6()
#plot7()
#plot8()
plot9()

