import numpy as np
import matplotlib.pyplot as plt
import sympy

from user_interface.matrixIO import read_data_from_txt
from user_interface.plotUtils import set_ax_equal, Arrow3D

def plot_path(ax):
    line = read_data_from_txt("build/data/look_ahead/" + "line_pnt").transpose()
    ax.plot(*line)
    ax.scatter(*line)

    curve = read_data_from_txt("build/data/look_ahead/" + "curve_pnt").transpose()
    ax.plot(*curve)

    set_ax_equal(ax)
    return

def calc_bezier_integral():
    t = sympy.symbols("t")
    x = sympy.symbols('x_0:6')
    y = sympy.symbols('y_0:6')
    z = sympy.symbols('z_0:6')
    xt = (1-t)**5 * x[0] + 5*(1-t)**4*t * x[1] + 10*(1-t)**3*t**2 * x[2] + \
            10*(1-t)**2*t**3 * x[3] + 5*(1-t)*t**4 * x[4] + t**5 * x[5]
    yt = (1-t)**5 * x[0] + 5*(1-t)**4*t * y[1] + 10*(1-t)**3*t**2 * y[2] + \
            10*(1-t)**2*t**3 * y[3] + 5*(1-t)*t**4 * y[4] + t**5 * y[5]
    zt = (1-t)**5 * z[0] + 5*(1-t)**4*t * z[1] + 10*(1-t)**3*t**2 * z[2] + \
            10*(1-t)**2*t**3 * z[3] + 5*(1-t)*t**4 * z[4] + t**5 * z[5]
    xdt = sympy.diff(xt, t)
    xdt2 = xdt * xdt
    ydt = sympy.diff(yt, t)
    ydt2 = ydt * ydt
    zdt = sympy.diff(zt, t)
    zdt2 = zdt * zdt

    dst = sympy.sqrt(xdt2 + ydt2 + zdt2)
    inte = sympy.integrate(dst, t)
    #  sympy.pprint(inte)
    print(inte)
    return


if __name__ == "__main__":
    #  calc_bezier_integral()
    savePath = "build/data/swing/"
    fig = plt.figure('空间三角形',figsize=(8,6))
    ax =plt.subplot(111,projection='3d')

    plot_path(ax)

    plt.show()

