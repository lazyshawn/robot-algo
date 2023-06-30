import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from user_interface.matrixIO import read_data_from_txt

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def plot_obstacle(ax):
    fname = "build/data/prm_scene"
    with open(fname, 'r') as file:
        line = file.readline()
        length, width = map(float, line.strip().split())
        ax.set_xlim(0, length)
        ax.set_ylim(0, width)
    points = read_data_from_txt(fname, skiprows=1)
    for i in range(len(points)):
        circle = plt.Circle((points[i,0], points[i,1]), points[i,2], alpha=0.5)
        ax.add_artist(circle)


def plot_connectioin(ax):
    fname = "build/data/prm_connection"
    points = read_data_from_txt(fname).transpose()
    if (len(points)):
        ax.scatter(points[0,:], points[1,:], label="Connection", facecolors='none', edgecolors='b')


def plot_guard(ax):
    fname = "build/data/prm_guard"
    points = read_data_from_txt(fname).transpose()
    if (len(points)):
        ax.scatter(points[0,:], points[1,:], label="Guard", c='r')

def plot_edge(ax):
    fname = "build/data/prm_edge"
    points = read_data_from_txt(fname).transpose()
    if (len(points)):
        numEdge = int(len(points[0]) / 3)
        for i in range(numEdge):
            edge = points[:,3*i:3*i+3]
            ax.plot(edge[0,:], edge[1,:], color='k', lw=1)


if __name__ == "__main__":
    fig = plt.figure('空间三角形',figsize=(8,6))
    ax =plt.subplot(111)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    colorMap = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
    #  plt.axis("equal")
    ax.set_aspect(1)

    plot_obstacle(ax)
    plot_edge(ax)
    plot_connectioin(ax)
    plot_guard(ax)

    ax.legend()
    plt.show()

