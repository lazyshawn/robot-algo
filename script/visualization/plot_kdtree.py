import numpy as np
import matplotlib.pyplot as plt

def plot_curve(ax):
    # 读取 cpp 生成的测试数据，在项目根目录下运行`script/..`
    mat = np.loadtxt("build/data/kdtree_points").transpose()
    mid = int(len(mat[0])/2 - 1)
    vecLen = np.linalg.norm(mat[:,0] - mat[:,1]);
    # 绘制离散点
    ax.scatter(mat[0,:], mat[1,:], mat[2,:], c = 'r', marker = 's', s = 10, label='Raw points')

    # 绘制质心和主轴方向
    vec = np.loadtxt("build/data/kdtree_pca").transpose()
    for i in range(0,3):
        ax.quiver(mat[0,mid], mat[1,mid], mat[2,mid], vec[0,i], vec[1,i], vec[2,i], length=vecLen, normalize=True, color=colorMap[i])

    # 绘制离散点
    out = np.loadtxt("build/data/kdtree_out").transpose()
    k = len(out[0] - 1)
    ax.scatter(out[0,:], out[1,:], out[2,:], c = 'c', marker = 's', s = 10, label='Target')
    for ii in range(1,k):
        ax.plot([out[0,0], out[0,ii]], [out[1,0], out[1,ii]], [out[2,0], out[2,ii]], c = 'y', dashes = [2,2])

if __name__ == "__main__":
    fig = plt.figure('空间三角形',figsize=(8,6))
    ax =plt.subplot(111,projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    colorMap = ['r', 'g', 'b', 'c', 'm', 'y', 'k']

    plot_curve(ax)

    ax.legend()
    plt.show()

