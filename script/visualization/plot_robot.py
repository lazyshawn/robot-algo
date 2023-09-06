from user_interface import *

import json, math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Patch

def axis_angle_to_rotation(axis, angle):
    axis = np.array(axis) / np.linalg.norm(axis)
    skewSymMat = np.mat([[0, -1*axis[2], axis[1]], [axis[2], 0, -1*axis[0]], [-1*axis[1], axis[0], 0]])
    rotMat = np.eye(3) + math.sin(angle) * skewSymMat + (1 - math.cos(angle)) * skewSymMat * skewSymMat

    return rotMat

class Robot():
    '''
    :Brief: 储存机械臂点云与配置信息
    '''
    linkPoint, linkOffset, linkPose, jointAxis = [], [], [], []
    numLink, numJoint = 0, 0

    def __init__(self, linkPoint, linkOffset, jointAxis):
        '''
        :Brief: 默认构造
        '''
        # 连杆点云数据
        self.linkPoint = linkPoint
        # 连杆偏移量 / 连杆参考点坐标
        self.linkOffset = linkOffset
        # 关节轴
        self.jointAxis = jointAxis
        self.numLink = len(linkOffset)
        self.numJoint = len(jointAxis)
        # 计算初始连杆的位姿
        self.update_link_pose()
        return

    @classmethod
    def load_from_file(cls, robotDir):
        '''
        :Brief: 从机械臂文件夹读取点云和配置文件
        :@robotDir: 机器人配置文件夹，包括配置文件、碰撞体文件
        '''
        # 读取机械臂配置文件
        with open(robotDir+"/config.json") as file:
            config = json.load(file)
            #  print(json.dumps(config,indent=2))
        # 获取关节和连杆字段
        poeJoint, collisionLink = config["PoE_joint"], config["collision_link"]
        # 关节数和连杆数
        numJoint, numLink = len(poeJoint), len(collisionLink)

        # 读取连杆偏移量
        linkOffset = []
        for i in range(numLink):
            tmpPos = np.array(collisionLink[i]["position"]).reshape(3,1)
            linkOffset.append(tmpPos)

        # 读取关节方向
        jointAxis = []
        for i in range(numJoint):
            jointAxis.append(np.array(poeJoint[i]["rotation_axis"]))

        # 读取连杆点云数据
        linkPoint = []
        for i in range(numLink):
            fname = robotDir+"/link_" + str(i) + ".ply"
            data = np.loadtxt(fname, skiprows=8).transpose() * 1000
            linkPoint.append(data)

        return cls(linkPoint, linkOffset, jointAxis)

    def update_link_pose(self, theta=[]):
        '''
        :Brief: 根据关节角更新连杆位姿
        '''
        self.linkPose = [np.eye(4,4) for i in range(self.numLink)]
        # 默认关节角
        if (len(theta) == 0):
            theta = [0 for i in range(self.numJoint)]
        # 连杆初始位姿
        for i in range(self.numLink):
            self.linkPose[i][0:3,0:3] = np.eye(3,3)
            self.linkPose[i][0:3,3] = self.linkOffset[i].reshape(1,3)

        # 第 i 个关节的运动对各个连杆产生的变换
        rotMat = np.eye(3,3)
        for i in range(self.numJoint):
            rotMat = axis_angle_to_rotation(self.jointAxis[i], theta[i]) * rotMat
            self.linkPose[i+1][0:3,0:3] = rotMat

    def plot(self, ax):
        handle = []
        for i in range(self.numLink):
            point = self.linkPoint[i]
            point = np.matmul(self.linkPose[i][0:3,0:3], point) + self.linkPose[i][0:3,3].reshape(3,1)
            handle.append(ax.scatter(*[point[coord,:] for coord in range(3)], s=2, label="link_"+str(i)))
        # 基坐标系
        coordLen = 1000
        plotUtils.plotCoordinate(ax, self.linkPose[0][0:3,0:3], coordLen, self.linkPose[0][0:3,3])
        plotUtils.plotCoordinate(ax, self.linkPose[self.numLink-1][0:3,0:3], coordLen, self.linkPose[self.numJoint][0:3,3])
        return handle


if __name__ == "__main__":
    fig = plt.figure('plotUtils test',figsize=(8,6))
    ax =plt.subplot(projection='3d')
    # 系统默认的颜色顺序. Ref: https://stackoverflow.com/questions/42086276
    #  colorMap = [p['color'] for p in plt.rcParams['axes.prop_cycle']]
    colorMap = plt.get_cmap("tab10")

    robot = Robot.load_from_file("data/r2700")
    robot.update_link_pose([0,0,0,0,0,math.pi/2])
    handle = robot.plot(ax)

    plotUtils.set_ax_equal(ax)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    ax.legend(handles=[Patch(facecolor=handle[i].get_facecolors()[0].tolist()) for i in range(7)],
              labels=["link_"+str(i) for i in range(7)])

    plt.show()

