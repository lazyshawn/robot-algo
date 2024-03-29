import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


class Arrow3D(FancyArrowPatch):
    '''
    @brief: 绘制三维箭头
    @param: [begX, endX], [begY, endY], [begZ, endZ] - 始末点
    @param: mutation_scale - float - 箭头比例
    @param: arrowstype - 箭头形状
    @param: lw - float - 线宽
    @ref  : https://stackoverflow.com/a/74986065
    @ref  : https://matplotlib.org/stable/api/_as_gen/matplotlib.patches.FancyArrowPatch.html
    '''
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        return np.min(zs)


def set_ax_equal(ax, zaxis=True):
    '''
    @brief: Make axes of 3d plot have equal scale so that geometry keep the
            original shape.
    @ref  : https://stackoverflow.com/a/31364297/8277667
    '''
    if (zaxis):
        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
    else:
        x_limits = ax.get_xlim()
        y_limits = ax.get_ylim()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)

    # The plot bounding box is a sphere in the sence of the infinity norm
    if (zaxis):
        z_limits = ax.get_zlim3d()
        z_range = abs(z_limits[1] - z_limits[0])
        plot_radius = 0.5 * max([x_range, y_range, z_range])
        z_middle = np.mean(z_limits)
        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
    else:
        plot_radius = 0.5 * max([x_range, y_range])
        ax.set_xlim([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim([y_middle - plot_radius, y_middle + plot_radius])


    # Ref: https://stackoverflow.com/a/73749399
    ax.set_aspect('equal')


def plotCoordinate(ax, coord = [[1,0,0],[0,1,0],[0,0,1]], len = 1, origin = [0,0,0]):
    '''
    @brief: 绘制坐标轴
    @param: ax - 绘图坐标轴
    @param: coord - 需要绘制的坐标轴矩阵
    @param: len - 绘制的坐标轴长度
    @param: origin - 原点坐标
    '''
    # 原点坐标
    origin = np.array(origin, dtype=float)
    # 坐标轴长度
    len = float(len)
    # 坐标轴矩阵
    coord = np.array(coord, dtype=float)
    # 坐标轴方向
    xdir = np.array([coord[0,0], coord[1,0], coord[2,0]], dtype = float)
    ydir = np.array([coord[0,1], coord[1,1], coord[2,1]], dtype = float)
    zdir = np.array([coord[0,2], coord[1,2], coord[2,2]], dtype = float)
    # 单位化
    xdir, ydir, zdir = xdir/np.linalg.norm(xdir)*len, ydir/np.linalg.norm(ydir)*len, zdir/np.linalg.norm(zdir)*len

    # 坐标轴终点
    x = np.array([origin[i]+xdir[i] for i in range(3)])
    y = np.array([origin[i]+ydir[i] for i in range(3)])
    z = np.array([origin[i]+zdir[i] for i in range(3)])
    ax.scatter(*[[x[i],y[i],z[i]] for i in range(3)], s=0)
    ax.scatter(*[origin[i] for i in range(3)])

    # 绘制坐标轴
    xAxis = Arrow3D([origin[0], x[0]], [origin[1], x[1]], [origin[2], x[2]], mutation_scale=20, lw=1, arrowstyle="-|>", color="r")
    ax.add_artist(xAxis)
    yAxis = Arrow3D([origin[0], y[0]], [origin[1], y[1]], [origin[2], y[2]], mutation_scale=20, lw=1, arrowstyle="-|>", color="g")
    ax.add_artist(yAxis)
    zAxis = Arrow3D([origin[0], z[0]], [origin[1], z[1]], [origin[2], z[2]], mutation_scale=20, lw=1, arrowstyle="-|>", color="b")
    ax.add_artist(zAxis)


def plot_mesh_from_off(fname, ax):
    '''
    @brief: 根据 off 文件绘制 mesh 模型
            主要要删除文件中的空行
    @param: fname - 文件名称
    @param: ax - 绘制的坐标轴
    '''
    # 打开文件
    with open(fname, 'r') as file:
        lines = file.readlines()
    if lines[0].strip() != "OFF":
        raise ValueError("Invalid OFF file")

    # 读取顶点、表面三角形
    numVertices, numFaces, _ = map(int, lines[1].strip().split())

    # 读取顶点坐标
    vertices = []
    for i in range(numVertices):
        vertex = list(map(float, lines[2+i].strip().split()))
        vertices.append(vertex)

    # 读取表面
    faces = []
    for i in range(numFaces):
        face = list(map(int, lines[2+numVertices+i].strip().split()[1:]))
        faces.append(face)

    # 绘制边缘点 (要先于绘制三角形，保证模型处在视野中央)
    boundingVertices = np.array([vertices[0]])
    for i in range(3):
        boundingVertices = np.vstack((boundingVertices, np.array(min(vertices, key = lambda p: p[i]))))
        boundingVertices = np.vstack((boundingVertices, np.array(max(vertices, key = lambda p: p[i]))))
    ax.scatter(boundingVertices[:,0], boundingVertices[:,1], boundingVertices[:,2], s=0)

    # 表面转换为三角形
    triangles = []
    for face in faces:
        triangle = [vertices[face[0]], vertices[face[1]], vertices[face[2]]]
        triangles.append(triangle)

    mesh = Poly3DCollection(triangles, alpha = 0.3)
    ax.add_collection3d(mesh)


if __name__ == "__main__":
    fig = plt.figure('plotUtils test',figsize=(8,6))
    ax =plt.subplot(projection='3d')

    a = Arrow3D([0, 1], [0, 1], [0, 1], mutation_scale=20, lw=1, arrowstyle="-|>", color="r")

    ax.add_artist(a)
    plt.show()

