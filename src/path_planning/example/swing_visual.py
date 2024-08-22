import numpy as np
import matplotlib.pyplot as plt

from user_interface.matrixIO import read_data_from_txt
from user_interface.plotUtils import set_ax_equal, Arrow3D

def plot_traj(ax, dataFile = "pos_dir"):
    mat = read_data_from_txt(savePath+dataFile).transpose()
    ax.plot(mat[0,:], mat[1,:], mat[2,:])

    # 箭头个数
    numArr = 20
    for i in np.linspace(0, len(mat[0])-1, numArr):
        i = round(i)
        arr = [[mat[j,i], mat[j,i] + mat[j+3,i]] for j in range(3)]
        a = Arrow3D(*arr, mutation_scale=20, lw=1, arrowstyle="-|>", color="r")
        ax.add_artist(a)
    set_ax_equal(ax_mbs)

def plot_inf(ax):
    t = np.linspace(0, np.pi/2, 5000000)
    a = 1
    x = a * np.sin(t)*np.cos(t) / (1 + np.sin(t)*np.sin(t))
    y = a * np.cos(t) / (1 + np.sin(t)*np.sin(t))
    #  print(1 / max(x))
    ax.scatter(x,y)

    set_ax_equal(ax, False)
    return


if __name__ == "__main__":
    savePath = "build/data/swing/"
    fig = plt.figure('空间三角形',figsize=(8,6))
    ax_mbs =plt.subplot(111,projection='3d')
    #  ax_inf =plt.subplot(111)

    plot_traj(ax_mbs)
    #  plot_inf(ax_inf)

    plt.show()

