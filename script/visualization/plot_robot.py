from user_interface import *

import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    fig = plt.figure('plotUtils test',figsize=(8,6))
    ax =plt.subplot(projection='3d')
    # 系统默认的颜色顺序. Ref: https://stackoverflow.com/questions/42086276
    #  colorMap = [p['color'] for p in plt.rcParams['axes.prop_cycle']]
    colorMap = plt.get_cmap("tab10")

    theta = np.array([90,45,0,0,0,0]) * np.pi / 180

    robot = kinematics.RobotBase("data/cloos/config.json")
    robot.load_obj_model("data/cloos/", "J")
    print(robot.solve_forward_kinematics(theta))

    robot.plot_model(ax, theta)

    plt.show()

