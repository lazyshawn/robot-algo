import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from plot_gjk import plot_simplex, plot_triangle

if __name__ == "__main__":
    fig = plt.figure('空间三角形',figsize=(8,6))
    ax =plt.subplot(111,projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # 读取 cpp 生成的测试数据，在项目根目录下运行`script/..`
    mat = np.loadtxt("build/data/qkhull_ga_points").transpose()
    x = mat[0,:]
    y = mat[1,:]
    z = mat[2,:]
    ax.scatter(x,y,z)
    #  tetrahedron = mat[:,-4:]
    #  plot_simplex(ax, tetrahedron)

    # 凸包数据
    #  mat = np.loadtxt("build/data/qkhull_hull").transpose()
    #  num = int(len(mat[0])/3)
    #  for i in range(num):
    #      plot_triangle(ax, mat[:,i*3 : (i+1)*3], 'green')

    plt.show()

