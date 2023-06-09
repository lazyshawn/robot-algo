
import numpy as np
import matplotlib.pyplot as plt

def plot_curve(ax):
    # 读取 cpp 生成的测试数据，在项目根目录下运行`script/..`
    mat = np.loadtxt("build/data/pca_points").transpose()
    vecLen = np.linalg.norm(mat[:,0] - mat[:,1]);
    # 绘制离散点
    ax.scatter(mat[0,:], mat[1,:], mat[2,:], c = 'r', marker = 's', s = 10, label='Raw points')
    # 绘制质心和主轴方向
    vec = np.loadtxt("build/data/pca_output").transpose()
    ax.scatter(vec[0,0], vec[1,0], vec[2,0], c = 'c', marker = '.', s = 30, label='C.O.M.')
    for i in range(1,4):
        ax.quiver(vec[0,0], vec[1,0], vec[2,0], vec[0,i], vec[1,i], vec[2,i], length=vecLen, normalize=True, color=colorMap[i-1])

if __name__ == "__main__":
    fig = plt.figure('空间三角形',figsize=(8,6))
    ax =plt.subplot(111,projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    #  colorMap = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
    colorMap = ['r', 'g', 'b', 'c', 'm', 'y', 'k']

    plot_curve(ax)

    ax.legend()
    plt.show()

