import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

if __name__ == "__main__":
    fig = plt.figure('空间三角形',figsize=(8,6))
    ax =plt.subplot(111,projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    colorMap = ['b', 'g', 'r', 'c', 'm', 'y', 'k']

    # 读取 cpp 生成的测试数据，在项目根目录下运行`script/..`
    mat = np.loadtxt("build/qkhull_points").transpose()
    num = len(mat[0])

    # 绘制商人初始城市
    #  for i in range(num):
        #  ax.scatter(mat[0,i],mat[1,i],mat[2,i], c=colorMap[i], s=60, marker='s')
    ax.plot(mat[0,:], mat[1,:], mat[2,:])

    #  # 绘制城市
    #  x = mat[0,numSalemen:]
    #  y = mat[1,numSalemen:]
    #  z = mat[2,numSalemen:]
    #  ax.scatter(x,y,z, s=10)
    #
    #  # 绘制商人的路径
    #  begNodeIdx = 0
    #  salemenIdx = 0
    #  for ii in range(numNode):
    #      # 跳过访问0个城市的商人
    #      while (salemenIdx < numSalemen and vistNum[salemenIdx] == 0):
    #          salemenIdx += 1
    #      # 起始点
    #      if (ii == begNodeIdx):
    #          x = [mat[0, salemenIdx], mat[0, ii+numSalemen]]
    #          y = [mat[1, salemenIdx], mat[1, ii+numSalemen]]
    #          z = [mat[2, salemenIdx], mat[2, ii+numSalemen]]
    #          ax.plot(x,y,z, c=colorMap[salemenIdx])
    #          begNodeIdx += vistNum[salemenIdx]
    #          salemenIdx += 1
    #      else:
    #          x = [mat[0, ii-1+numSalemen], mat[0, ii+numSalemen]]
    #          y = [mat[1, ii-1+numSalemen], mat[1, ii+numSalemen]]
    #          z = [mat[2, ii-1+numSalemen], mat[2, ii+numSalemen]]
    #          ax.plot(x,y,z, c=colorMap[salemenIdx-1])
    #
    #
    plt.show()

