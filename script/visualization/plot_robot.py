from user_interface import *

import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    fig = plt.figure('plotUtils test',figsize=(8,6))
    ax =plt.subplot(projection='3d')

    theta = np.array([0,-45,45,0,90,0]) * np.pi / 180

    robot = kinematics.RobotBase("data/cloos/")
    robot.load_obj_model()
    #  print(robot.solve_forward_kinematics(theta))

    robot.plot_model(ax, theta)

    plt.show()

