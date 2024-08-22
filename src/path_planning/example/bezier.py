import numpy as np
import matplotlib.pyplot as plt
import sympy

from user_interface.matrixIO import read_data_from_txt
from user_interface.plotUtils import set_ax_equal, Arrow3D

def plot_path(ax):
    # 绘制节点
    line = read_data_from_txt("build/data/bezier/" + "node_pnt").transpose()
    if (len(line) > 3):
        # 曲线节点
        ax.scatter(line[0,:], line[1,:], line[2,:])
        # 
        for i in range(len(line[0])):
            vctLen = 0.8
            pkg = [[line[j,i], line[j,i]+line[j+3, i]*vctLen] for j in range(3)]
            a = Arrow3D(*pkg, mutation_scale=20, lw=1, arrowstyle="-|>", color="r")
            ax.add_artist(a)
    else:
        ax.plot(*line)
        ax.scatter(*line)

    # 绘制曲线
    curve = read_data_from_txt("build/data/bezier/" + "curve_pnt").transpose()
    ax.plot(*curve)

    # 绘制采样点
    sample = read_data_from_txt("build/data/bezier/" + "sample_pnt").transpose()
    ax.scatter(*sample)

    set_ax_equal(ax)
    return


if __name__ == "__main__":
    savePath = "build/data/swing/"
    fig = plt.figure('空间三角形',figsize=(8,6))
    ax =plt.subplot(111,projection='3d')

    plot_path(ax)

    plt.show()

