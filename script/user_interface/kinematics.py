import json, math
import numpy as np
from scipy import linalg
from scipy.spatial.transform import Rotation
from scipy.stats import beta

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

class RobotBase:
    '''
    :Brief:
    '''
    jointAxis, jointLimit, linkPoint = [], [], []
    numLink, numJoint = 0, 0
    jointCouple = False
    m0 = np.eye(4)

    def __init__(self, fname):
        self.load_config(fname)

    def load_config(self, fname):
        # 读取机械臂配置文件
        with open(fname) as file:
            config = json.load(file)
            self.numJoint = config["num_of_joint"]
            self.numLink = len(config["collision_link"])
            self.jointCouple = config["j2_j3_coupling"]

            # 关节限位
            self.jointLimit.append(np.array(config["joint_limits"]["positions"]["uppers"]) * math.pi / 180)
            self.jointLimit.append(np.array(config["joint_limits"]["positions"]["lowers"]) * math.pi / 180)

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

    def load_obj_model(self, robotDir):
        # 读取连杆点云数据
        for i in range(self.numLink):
            fname = robotDir+"/link_" + str(i) + ".ply"
            data = np.loadtxt(fname, skiprows=8).transpose() * 1000
            self.linkPoint.append(data)

        # 零位时连杆的位姿
        #  self.linkOffset = []
        #  for i in range(self.numLink):
        #      tmpPos = np.array(config["collisionLink"][i]["position"]).reshape(3,1)
        #      linkOffset.append(tmpPos)


    def solve_forward_kinematics(self, theta):
        # 2,3 关节耦合
        if(self.jointCouple):
            theta[2] = theta[2] + theta[1]
        trans = self.m0
        for i in range(self.numJoint):
            idx = self.numJoint - i - 1
            trans = np.matmul(lieSE3(self.jointAxis[idx] * theta[idx]), trans)
        return trans

    def plot_model(self, theta):
        trans = []
        tmpTrans = np.eye(4)
        for i in range(self.numLink):
            idx = self.numJoint - i - 1
            tmpTrans = np.matmul(lieSE3(self.jointAxis[idx] * theta[idx]), tmpTrans)
            trans.append(tmpTrans)
            return

        return

if __name__ == "__main__":
    print("This is kinematics.py")
    robot = RobotBase("data/r2700/config.json")
    robot.load_obj_model("data/r2700")
    #  print(robot.solve_forward_kinematics(np.array([0,20,30,90,60,0]) * math.pi / 180))

