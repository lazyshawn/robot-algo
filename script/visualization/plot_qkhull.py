import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def plot_triangle(ax, triangle, color_='red'):
    '''
    :triangle: [p1,p2,...pn] 点的列向量组合
    Ref: https://ask.csdn.net/questions/7628135
    '''
    if (not int(len(triangle.transpose())) == 3):
        print("Warning: over vertex in a triangle, only show first 3 points.")
    # 只绘制前三个点
    x = triangle[0,0:3]
    y = triangle[1,0:3]
    z = triangle[2,0:3]
    # 散点图
    ax.scatter(x,y,z, color = color_)
    # 绘制边
    ax.plot(x,y,z,'black')
    # 边封闭
    ax.plot(x[[0,-1]],y[[0,-1]],z[[0,-1]],'black')
    # 三角形表面
    points = [list(zip(x, y, z))]
    triangle = Poly3DCollection(points, alpha=0.25, facecolor=color_)
    # 绘制三角形表面
    ax.add_collection3d(triangle)
    return

def plot_simplex(ax, simplex, color_='red'):
    '''
    :simplex: 单纯形顶点
    '''
    n = int(len(simplex.transpose()))
    if (not (n == 3 or n == 4)):
        print("Error: wrong number of vertex in a simplex. (3 or 4)")
        return
    if (n==3):
        plot_triangle(ax, simplex, color_)
        return
    plot_triangle(ax, simplex[:,[1,2,3]], color_)
    plot_triangle(ax, simplex[:,[0,2,3]], color_)
    plot_triangle(ax, simplex[:,[0,1,3]], color_)
    plot_triangle(ax, simplex[:,[0,1,2]], color_)
    return

if __name__ == "__main__":
    fig = plt.figure('空间三角形',figsize=(8,6))
    ax =plt.subplot(111,projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # 读取 cpp 生成的测试数据，在项目根目录下运行`script/..`
    mat = np.loadtxt("build/data/qkhull_points").transpose()
    print(mat)

    x = mat[0,:-4]
    y = mat[1,:-4]
    z = mat[2,:-4]
    ax.scatter(x,y,z)

    ans = mat[:,-4:]
    print(ans)
    plot_simplex(ax, ans)
    #  plot_simplex(ax, setA)
    #  plot_simplex(ax, setB, 'blue')

    plt.show()


