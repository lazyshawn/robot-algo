import sys, os, math
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from user_interface.matrixIO import read_data_from_txt
from user_interface.plotUtils import set_ax_equal

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def axis_angle(axis, angle):
    axis = np.array(axis[0:4])
    axis = axis / np.linalg.norm(axis)

    kx = np.zeros((3,3))
    kx[0][1], kx[0][2] = -axis[2], axis[1]
    kx[1][0], kx[1][2] = axis[2], -axis[0]
    kx[2][0], kx[2][1] = -axis[1], axis[0]

    eye = np.eye(3)
    rot = eye + np.sin(angle)*kx + (1-np.cos(angle))*kx@kx

    return rot

def plot_circle(ax, p1, p2, p3):
    p1, p2, p3 = [np.array(p1), np.array(p2), np.array(p3)]

    a = p1 - p2
    b = p3 - p2
    # 半径过大时，直接绘制直线
    if (np.cross(a,b).dot(np.cross(a,b)) < 1e-10):
        ax.plot([p1[0], p3[0]], [p1[1], p3[1]], [p1[2], p3[2]])
        return
    r = np.cross(a.dot(a)*b - b.dot(b)*a, np.cross(a,b)) / (2*np.cross(a,b).dot(np.cross(a,b)))
    #  print(math.sqrt(r.dot(r)))

    center = r + p2
    #  print("center = ", center)
    op1, op2, op3 = p1-center, p2-center, p3-center

    # 法向
    norm = np.cross(op1, op3)
    norm = norm / np.linalg.norm(norm)

    # O_p1 与 O_p3 的夹角
    theta = np.arccos(op1.dot(op3) / np.linalg.norm(op1) / np.linalg.norm(op3))
    # p2 是否在 <p1_O_p3 的劣弧内
    #  if (np.cross(op1, op2).dot(np.cross(op2,op3)) < 0):
    if (op1.dot(op2) <=0 or op2.dot(op3) <= 0):
        theta = 2*np.pi - theta
        norm *= -1
    #  print("norm = ", norm)
    #  print(theta * 180 / np.pi)

    #  ax.scatter([p1[0],p2[0],p3[0]], [p1[1],p2[1],p3[1]], [p1[2],p2[2],p3[2]])

    q = np.linspace(0, theta, 50)
    x, y, z = np.array([]), np.array([]), np.array([])
    for i in q:
        rot = axis_angle(norm, i)
        pnt = rot@op1 + center
        x, y, z = np.append(x, pnt[0]), np.append(y, pnt[1]), np.append(z, pnt[2])
    ax.plot(x, y, z)


def plot_mbs(ax, dataFile="final_22.txt"):
    # 读取 cpp 生成的测试数据，在项目根目录下运行`script/..`
    mat = read_data_from_txt(dataPath+dataFile).transpose()
    idx = read_data_from_txt(savePath+"sep_idx")
    apIdx = read_data_from_txt(savePath+"arc_point_idx")

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # 原始数据图
    ax.plot(mat[0,:], mat[1,:], mat[2,:], linestyle = ':')

    # 分割点
    begIdx = int(idx[0][0])
    for i in range(len(idx[0])):
        endIdx = int(i+1) % len(idx[0])
        x = [mat[0, int(idx[0][i])], mat[0, int(idx[0][endIdx])]]
        y = [mat[1, int(idx[0][i])], mat[1, int(idx[0][endIdx])]]
        z = [mat[2, int(idx[0][i])], mat[2, int(idx[0][endIdx])]]
        #  ax.plot(x, y, z, c='r')
        if (len(idx[0]) < 15):
            ax.scatter(mat[0, int(idx[0][i])],mat[1, int(idx[0][i])],mat[2, int(idx[0][i])])

    #  for i in [0]:
    for i in range(len(idx[0])):
        begIdx = int(idx[0][i])
        if (i == len(idx[0])-1):
            break
        midIdx = int(apIdx[0][i])
        endIdx = int(idx[0][i+1])
        plot_circle(ax, mat[:, begIdx], mat[:, midIdx], mat[:, endIdx])

        begIdx = endIdx

    set_ax_equal(ax)


def plot_mpc(ax):
    pnt = read_data_from_txt(savePath+"mpc").transpose()
    idx = read_data_from_txt(savePath+"mpc_dp_idx")
    ax.plot(pnt[0,:], pnt[1,:])
    ax.scatter(pnt[0,:], pnt[1,:])

    for i in range(len(idx[0])):
        if (i < len(idx[0])-1):
            x = [pnt[0, int(idx[0][i])], pnt[0, int(idx[0][i+1])]]
            y = [pnt[1, int(idx[0][i])], pnt[1, int(idx[0][i+1])]]
            ax.plot(x, y)
        ax.scatter(pnt[0, int(idx[0][i])],pnt[1, int(idx[0][i])], c='r')

def plot_trans(ax):
    pnt = read_data_from_txt(savePath+"trans_pnt").transpose()
    ax.plot(*pnt)

    set_ax_equal(ax)
    return

def plot_discrete_traj(ax):
    pnt = read_data_from_txt(savePath+"node_pnt").transpose()
    arc = read_data_from_txt(savePath+"arc_info").transpose()
    ax.scatter(*pnt)
    for i in range(len(arc[0])):
        if arc[3,i] > 0:
            center = arc[0:3,i]
            op1 = pnt[:,i] - center
            theta = np.linalg.norm(arc[4:,i])
            norm = arc[4:,i] / theta
            q = np.linspace(0, theta, 50)
            x, y, z = np.array([]), np.array([]), np.array([])
            for i in q:
                rot = axis_angle(norm, i)
                pnt = rot@op1 + center
                x, y, z = np.append(x, pnt[0]), np.append(y, pnt[1]), np.append(z, pnt[2])
            ax.plot(x, y, z, c = '#1f77b4', linestyle = ':')
        else:
            pkg = [[pnt[j,i], pnt[j,i+1]] for j in range(3)]
            ax.plot(*pkg, c = '#1f77b4', linestyle = ':')

    set_ax_equal(ax)
    return

if __name__ == "__main__":
    dataPath = "data/curve_decomposition/"
    savePath = "build/data/curve_decomposition/"

    dataFile = sys.argv[1] if (len(sys.argv) > 1) else "final_22.txt"

    fig = plt.figure('空间三角形',figsize=(8,6))
    ax_mbs =plt.subplot(211,projection='3d')
    ax_mpc =plt.subplot(212)

    #  ax_mbs =plt.subplot(211,projection='3d')
    #  ax_trans =plt.subplot(111,projection='3d')
    #  plot_discrete_traj(ax_mbs)
    #  plot_trans(ax_mbs)

    plot_mbs(ax_mbs, dataFile)
    plot_mpc(ax_mpc)


    plt.show()

