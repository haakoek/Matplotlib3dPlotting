# -*- coding: utf-8 -*-
"""
Created on Sat Sep  1 21:47:59 2018

@author: Haakon
"""

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation

def addVector(ax,P,P0=np.zeros(3),c='r'):
    ax.plot([P0[0],P[0]],[P0[1],P[1]],[P0[2],P[2]],color = c) #Draw line from P0 to P
    ax.scatter([P[0]], [P[1]], [P[2]], color=c, marker = "o")

# Attach 3D axis to the figure
fig = plt.figure()
ax  = p3.Axes3D(fig)

x0 = np.zeros(3)
x1 = np.array([0.2,0,0])
x2 = np.array([0,0.5,0.3])
x3 = np.cross(x1,x2)

ax.scatter([0], [0], [0], color='k', marker = "x")
ax.plot([-1,1],[0,0],[0,0],color='k') #Draw x-axis
ax.plot([0,0],[-1,1],[0,0],color='k') #Draw y-axis
ax.plot([0,0],[0,0],[-1,1],color='k') #Draw z-axis
#ax.plot([P0[0],P[0]],[P0[1],P[1]],[P0[2],P[2]],color = c) #Draw line from P0 to P 
#ax.plot([P0[0],P[0]],[P0[1],P[1]],[P0[2],P[2]],color = c) #Draw line from P0 to P
       
addVector(ax,x1,c='r')
addVector(ax,x2,c='g')
addVector(ax,x3,c='b')


# Setthe axes properties
ax.set_xlim3d([-1, 1])
ax.set_xlabel('X')

ax.set_ylim3d([-1.0, 1.0])
ax.set_ylabel('Y')

ax.set_zlim3d([-1.0, 1.0])
ax.set_zlabel('Z')

ax.set_title('Vectors in R^3')


plt.show()