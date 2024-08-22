from user_interface import *

import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    fig = plt.figure('plotUtils test',figsize=(8,6))
    ax =plt.subplot(projection='3d')

    #  theta = np.array([0,-45,45,0,90,0]) * np.pi / 180
    #  theta = np.array([0,0,0,45,0,0]) * np.pi / 180
    #  theta = np.array([-5.136, -7.4044, 34.2832, -0.2824, 65.9928, -43.9743]) * np.pi / 180
    theta = np.array([10,10,10,10,10,10]) * np.pi / 180

    #  robot = kinematics.RobotBase("data/cloos/")
    robot = kinematics.RobotBase("data/efort/")
    robot.load_obj_model()
    #  print(robot.solve_forward_kinematics(theta))

    #  robot.plot_model(ax, theta)
    robot.plot_tcp(ax, theta, 500)

    plt.show()

