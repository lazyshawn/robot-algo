import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def plot_curve(ax):
    # 读取 cpp 生成的测试数据，在项目根目录下运行`script/..`
    mat = np.loadtxt("build/data/nurbs_curve_output").transpose()
    # 绘制 NURBS 曲线
    ax.plot(mat[0,:], mat[1,:], mat[2,:], label='NURBS curve')

    points = np.loadtxt("build/data/nurbs_curve_ctrlpoint").transpose()
    print(points)
    ax.scatter(points[0,:], points[1,:], points[2,:], c = 'r', marker = 's', s = 30, label='Ctrl points')
    ax.plot(points[0,:], points[1,:], points[2,:], c = 'r', dashes=[1,1])

def plot_surface(ax):
    # 绘制控制点
    with open("build/data/nurbs_surface_ctrlpoint", "r") as f:
        numU = int(f.readline())
        numV = int(f.readline())
    ctrlPoints = np.loadtxt("build/data/nurbs_surface_ctrlpoint", skiprows=2)
    ax.scatter(ctrlPoints[:,0], ctrlPoints[:,1], ctrlPoints[:,2], c = 'r', marker = 's', s = 30)
    ctrlPoints = ctrlPoints.reshape((numU,numV,3))
    for i in range(numU):
        ax.plot(ctrlPoints[i,:,0], ctrlPoints[i,:,1], ctrlPoints[i,:,2], c = 'r', dashes = [1,1])
    for i in range(numV):
        ax.plot(ctrlPoints[:,i,0], ctrlPoints[:,i,1], ctrlPoints[:,i,2], c = 'm', dashes = [1,1])

    # 绘制控制点
    with open("build/data/nurbs_surface_output", "r") as f:
        sepU = int(f.readline())
        sepV = int(f.readline())
    outPoints = np.loadtxt("build/data/nurbs_surface_output", skiprows=2)
    outPoints = outPoints.reshape((sepU,sepV,3))
    for i in range(sepU):
        ax.plot(outPoints[i,:,0], outPoints[i,:,1], outPoints[i,:,2], c = 'c')
    for i in range(sepV):
        ax.plot(outPoints[:,i,0], outPoints[:,i,1], outPoints[:,i,2], c = 'c')

def plot_fit(ax):
    plot_curve(ax)
    points = np.loadtxt("build/data/nurbs_curve_fitpoint").transpose()
    print(points)
    ax.scatter(points[0,:], points[1,:], points[2,:], c = 'c', marker = 's', s = 30, label='Fitting points')

if __name__ == "__main__":
    fig = plt.figure('空间三角形',figsize=(8,6))
    ax =plt.subplot(111,projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    colorMap = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

    plot_fit(ax)
    #  plot_curve(ax)
    #  plot_surface(ax)

    ax.legend()
    plt.show()

