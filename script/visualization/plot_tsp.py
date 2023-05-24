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
    mat = np.loadtxt("build/data/qkhull_ga_seq_points").transpose()
    result = np.loadtxt("build/data/qkhull_ga_result").transpose()
    numNode = int(result[0])
    numSalemen = int(result[1])
    vistNum = result[-numSalemen:]
    vistSeq = result[2:-numSalemen]
    print(vistNum)
    print(vistSeq)

    # 绘制商人初始城市
    x = mat[0,0:numSalemen]
    y = mat[1,0:numSalemen]
    z = mat[2,0:numSalemen]
    ax.scatter(x,y,z, c='r', s=60, marker='*')

    # 绘制城市
    x = mat[0,numSalemen:]
    y = mat[1,numSalemen:]
    z = mat[2,numSalemen:]
    ax.scatter(x,y,z, s=10)

    colorMap = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
    # 绘制商人的路径
    begNodeIdx = 0
    salemenIdx = 0
    for ii in range(numNode):
        # 起始点
        if (ii == begNodeIdx):
            x = [mat[0, salemenIdx], mat[0, ii+numSalemen]]
            y = [mat[1, salemenIdx], mat[1, ii+numSalemen]]
            z = [mat[2, salemenIdx], mat[2, ii+numSalemen]]
            ax.plot(x,y,z, c=colorMap[salemenIdx])
            # 跳过访问0个城市的商人
            while (True):
                print(salemenIdx)
                begNodeIdx += vistNum[salemenIdx]
                salemenIdx += 1
                if (not vistNum[salemenIdx-1] == 0): break
        else:
            x = [mat[0, ii-1+numSalemen], mat[0, ii+numSalemen]]
            y = [mat[1, ii-1+numSalemen], mat[1, ii+numSalemen]]
            z = [mat[2, ii-1+numSalemen], mat[2, ii+numSalemen]]
            ax.plot(x,y,z, c=colorMap[salemenIdx-1])


    plt.show()

