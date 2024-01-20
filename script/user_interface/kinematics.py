import json
import numpy as np
from scipy import linalg
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

from user_interface import plotUtils

def lieso3(vec):
    mat = [[0, -vec[2], vec[1]], [vec[2], 0, -vec[0]], [-vec[1], vec[0], 0]]
    return np.array(mat, dtype=np.float64)

def lieSO3(vec):
    so = lieso3(vec)
    return linalg.expm(so)

def liese3(twist):
    v = twist[0:3]; w = twist[3:6]
    exponent = np.zeros((4,4))
    exponent[0:3, 0:3] = lieso3(w)
    exponent[0:3, 3] = v
    return exponent

def lieSE3(twist):
    return linalg.expm(liese3(twist))

def axis_angle_to_rotation(axis, angle):
    axis = np.array(axis) / np.linalg.norm(axis)
    skewSymMat = np.mat([[0, -1*axis[2], axis[1]], [axis[2], 0, -1*axis[0]], [-1*axis[1], axis[0], 0]])
    rotMat = np.eye(3) + np.sin(angle) * skewSymMat + (1 - np.cos(angle)) * skewSymMat * skewSymMat
    return rotMat

'''
@brief  描述机器人构型的基类
       1. 存储机器人关节信息，连杆碰撞体的点云信息
       1. 计算机器人正运动学，绘制机器人模型
@todo  多 TCP 配置文件的支持
'''
class RobotBase:
    jointAxis, jointLimit, linkPoint = [], [], []
    tran0 = []
    numLink, numJoint = 0, 0
    jointCouple = False
    m0 = np.eye(4)

    def __init__(self, fname):
        self.load_config(fname)

    '''
    @brief  读取机械臂配置文件
    @param  fname    配置文件路径
    '''
    def load_config(self, fname):
        # 打开配置文件
        with open(fname) as file:
            config = json.load(file)
            self.numJoint = config["num_of_joint"]
            self.numLink = len(config["collision_link"])
            self.jointCouple = config["j2_j3_coupling"]

            # 关节限位
            self.jointLimit.append(np.array(config["joint_limits"]["positions"]["uppers"]) * np.pi / 180)
            self.jointLimit.append(np.array(config["joint_limits"]["positions"]["lowers"]) * np.pi / 180)

            # 零位的 TCP 位姿 (w,x,y,z)
            qua = config["end-effector"]["quaternion"]
            rotMat = Rotation.from_quat((qua[3], qua[2], qua[1], qua[0]))
            self.m0[0:3, 0:3] = rotMat.as_matrix()
            self.m0[0:3, 3] = config["end-effector"]["position"]

            # 关节轴
            for joint in config["PoE_joint"]:
                pnt = np.array(joint["position"])
                dir = np.array(joint["rotation_axis"])
                dir = dir / np.linalg.norm(dir)
                self.jointAxis.append(np.concatenate([-1*np.cross(dir, pnt), dir]))
            #  print(self.jointAxis)

            # 读取连杆初始位姿
            collisionLink = config["collision_link"]
            for collis in collisionLink:
                qua = collis["quaternion"]
                rotMat = Rotation.from_quat((qua[3], qua[2], qua[1], qua[0]))
                tmpTran = np.eye(4)
                tmpTran[0:3, 0:3] = rotMat.as_matrix()
                tmpTran[0:3, 3] = collis["position"]
                self.tran0.append(tmpTran)

    '''
    @brief  加载连杆模型的点云数据，仅支持 ply 格式
    @param  robotDir    点云数据所在文件夹
    @param  prefix      点云文件的命名前缀
    @param  scale       点云单位缩放，确保点云单位和配置文件中的连杆偏移单位一致
    '''
    def load_obj_model(self, robotDir, prefix = "link_", scale = 1):
        for i in range(self.numLink):
            fname = robotDir + prefix + str(i) + ".ply"
            data = np.loadtxt(fname, skiprows=8).transpose() * scale
            self.linkPoint.append(data)

    '''
    @brief  求解正运动学
    @param  theta    各关节的关节角
    @return trans    TCP 相对于基坐标系的位姿
    '''
    def solve_forward_kinematics(self, theta):
        # 2,3 关节耦合
        if(self.jointCouple):
            theta[2] = theta[2] + theta[1]
        trans = self.m0
        for i in range(self.numJoint):
            idx = self.numJoint - i - 1
            trans = np.matmul(lieSE3(self.jointAxis[idx] * theta[idx]), trans)
        return trans

    '''
    @brief  绘制机器人模型
    @param  ax          matplotlib 的坐标轴
    @param  theta       各关节的关节角
    @param  coordLen    基坐标系和TCP坐标系的坐标轴长度
    '''
    def plot_model(self, ax, theta, coordLen = 0):
        handle = []
        # 机器人基座 - link_0
        point = self.linkPoint[0]
        point = np.matmul(self.tran0[0][0:3, 0:3], point) + self.tran0[0][0:3,3].reshape(3,1)
        handle.append(ax.scatter(*[point[coord,:] for coord in range(3)], s=2, label = "Base_link"))
        # 计算每个连杆的旋转矩阵
        tran = [np.eye(4) for i in range(self.numJoint)]
        # 第 i 个关节的运动
        for i in range(self.numJoint-1, -1, -1):
            # 第 j 个连杆
            for j in range(self.numJoint-1, i-1, -1):
                tran[j] = np.matmul(lieSE3(self.jointAxis[i] * theta[i]), tran[j])
        # 机器人连杆 - link_i
        for i in range(self.numJoint):
            # 初始位置
            point = self.linkPoint[i+1]
            # 影响当前连杆的转换矩阵
            tmpTrans = np.matmul(tran[i], self.tran0[i+1])
            # 当前连杆的位置
            point = np.matmul(tmpTrans[0:3, 0:3], point) + tmpTrans[0:3,3].reshape(3,1)
            handle.append(ax.scatter(*[point[coord,:] for coord in range(3)], s=2, label="link_"+str(i+1)))
            #  ax.scatter(*[point[coord,:] for coord in range(3)], s=2)

        if (coordLen > 0):
            # 基坐标系位姿
            plotUtils.plotCoordinate(ax, self.tran0[0][0:3,0:3], coordLen, self.tran0[0][0:3,3])
            # TCP 位姿
            tmpTrans = np.matmul(tran[self.numJoint-1], self.m0)
            plotUtils.plotCoordinate(ax, tmpTrans[0:3,0:3], coordLen, tmpTrans[0:3,3])

        plotUtils.set_ax_equal(ax)
        ax.legend(handles=[Patch(facecolor=handle[i].get_facecolors()[0].tolist()) for i in range(self.numLink)],
                  labels=[handle[i]._label for i in range(self.numLink)])
        return handle

if __name__ == "__main__":
    print("This is kinematics.py")
    theta = np.array([0,20,30,90,60,0]) * np.pi / 180

    fig = plt.figure('plotUtils test',figsize=(8,6))
    ax =plt.subplot(projection='3d')

    robot = RobotBase("data/r2700/config.json")
    robot.load_obj_model("data/r2700/", "link_", 1000)

    robot.plot_model(ax, theta, 1000)
    print(robot.solve_forward_kinematics(theta))

    plt.show()

