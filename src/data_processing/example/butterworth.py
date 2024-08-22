import sys, os, math
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from user_interface.matrixIO import read_data_from_txt
from user_interface.plotUtils import set_ax_equal

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def plot_butterworth(ax):
    pnt = read_data_from_txt(savePath+"raw").transpose()
    time = range(0,len(pnt[0]))
    ax.plot(time, pnt[0, :])
    ax.plot(time[:-5], pnt[1, 5:])
    return

if __name__ == "__main__":
    dataPath = "data/data_processing/"
    savePath = "build/data/data_processing/"

    dataFile = sys.argv[1] if (len(sys.argv) > 1) else "final_22.txt"

    fig = plt.figure('空间三角形',figsize=(8,6))
    #  ax_mbs =plt.subplot(211,projection='3d')
    ax =plt.subplot(111)

    plot_butterworth(ax)

    plt.show()
