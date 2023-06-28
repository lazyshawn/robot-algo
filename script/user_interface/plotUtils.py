import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d


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


if __name__ == "__main__":
    fig = plt.figure('plotUtils test',figsize=(8,6))
    ax =plt.subplot(projection='3d')

    a = Arrow3D([0, 1], [0, 1], [0, 1], mutation_scale=20, lw=1, arrowstyle="-|>", color="r")

    ax.add_artist(a)
    plt.show()

