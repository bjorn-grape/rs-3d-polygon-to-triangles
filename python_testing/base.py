import mpl_toolkits.mplot3d as a3
import matplotlib.colors as colors
import pylab as pl
import scipy as sp
import numpy as np


def make_plot(a, b):
    list_index = np.array(a)
    list_points = np.array(b)

    ax = a3.Axes3D(pl.figure())
    ax.set_xlim(min(list_points[..., 0]), max(list_points[..., 0]))
    ax.set_ylim(min(list_points[..., 1]), max(list_points[..., 1]))
    ax.set_zlim(min(list_points[..., 2]), max(list_points[..., 2]))
    for i, elm in enumerate(list_index):
        vtx = [list_points[elm[0]], list_points[elm[1]], list_points[elm[2]]]
        tri = a3.art3d.Poly3DCollection([vtx])
        tri.set_color(colors.rgb2hex(sp.rand(3)))
        tri.set_edgecolor('k')
        ax.add_collection3d(tri)

    # for angle in range(0, 360):
    #     ax.view_init(30, angle)
    #     pl.draw()
    #     pl.pause(.001)
    pl.draw()
    pl.pause(10)
