import json, math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Patch

from user_interface import *

def calc_double_s_interp_parameters(q0, q1, v0, v1, vMax, aMax, jMax):
    """
    @brief 双S曲线插补
    @notice 缩放系数: n*v, n*2a, n*3j

    @param q0 初始位置
    @param q1 结束位置
    @param v0 初始速度
    @param v1 结束速度
    @param vMax 最大速度，vMin = -vMax
    @param aMax 最大加速度，aMax = -aMax
    @param jMax 最大加加速度，jMax = -jMax

    @return param [Ta, Tv, Td, Tj1, Tj2, q0, q1, v0, v1, vMax, aMax, jMax]
    """

    """
    判断轨迹合理性
    """
    detQ = q1 - q0
    Tjs = min(math.sqrt(abs(v1 - v0) / jMax),2)
    #  if (Tjs < aMax/jMax and detQ < Tjs*(v0+v1)) or (Tjs >= aMax/jMax and detQ < (v0+v1)*(Tjs + abs(v1-v0)/aMax)/2):
    #      print("Error: trajectory not feasible.")
    #      return []
    if Tjs < aMax/jMax:
        if detQ < Tjs*(v0+v1):
            print("Error: trajectory not feasible.")
            return []
    else:
        if detQ < (v0+v1)*(Tjs + abs(v1-v0)/aMax)/2:
            print("Error: trajectory not feasible.")
            return []

    """
    计算各阶段的持续时间
    """
    # Case: vLim = vMax, 运动过程中达到最大速度
    # 加速
    if (vMax - v0)*jMax < aMax*aMax:
        Tj1 = math.sqrt((vMax-v0) / jMax)
        Ta = 2*Tj1
    else:
        Tj1 = aMax/jMax
        Ta = Tj1 + (vMax - v0) / aMax
    # 减速
    if (vMax - v1)*jMax < aMax*aMax:
        Tj2 = math.sqrt((vMax-v1) / jMax)
        Td = 2*Tj2
    else:
        Tj2 = aMax/jMax
        Td = Tj2 + (vMax-v1)/aMax

    Tv = (q1-q0)/vMax - Ta*(1+v0/vMax)/2 - Td*(1+v1/vMax)/2

    if Tv > 0:
        #  print("达到最大速度")
        pass
    # Case2: vLim < vMax, 运动过程中未达到最大速度
    else:
        #  print("未达到最大速度")
        Tv = 0.0
        Tj1 = Tj2 = aMax/jMax
        det = aMax**4 / jMax**2 + 2*(v0*v0 + v1*v1) + aMax*(4*(q1-q0) - 2*(v0+v1)*aMax/jMax)
        Ta = (aMax*aMax/jMax - 2*v0 + math.sqrt(det)) / (2*aMax)
        Td = (aMax*aMax/jMax - 2*v1 + math.sqrt(det)) / (2*aMax)

        while (Ta > 0 and Ta < 2*Tj1) or (Td > 0 and Td < 2*Tj2):
            aMax = 0.98 * aMax
            Tj1 = Tj2 = aMax/jMax
            det = aMax**4 / jMax**2 + 2*(v0*v0 + v1*v1) + aMax*(4*(q1-q0) - 2*(v0+v1)*aMax/jMax)
            Ta = (aMax*aMax/jMax - 2*v0 + math.sqrt(det)) / (2*aMax)
            Td = (aMax*aMax/jMax - 2*v1 + math.sqrt(det)) / (2*aMax)

        if Ta <= 0:
            Ta = Tj1 = 0
            Td = 2*(q1-q0)/(v1+v0)
            Tj2 = (jMax*(q1-q0) - math.sqrt(jMax*(jMax*(q1-q0)*(q1-q0) + (v1+v0)*(v1+v0)*(v1-v0)))) / (jMax*(v1+v0))
        if Td <= 0:
            Td = Tj2 = 0
            Ta = 2*(q1-q0)/(v1+v0)
            Tj1 = (jMax*(q1-q0) - math.sqrt(jMax*(jMax*(q1-q0)*(q1-q0) - (v1+v0)*(v1+v0)*(v1-v0)))) / (jMax*(v1+v0))

        if ((Ta > 0 and Ta < 2*Tj1) or (Td > 0 and Td < 2*Tj2)):
            print("加减速阶段不完整")
            return []
        else:
            Tj1 = 0 if Ta == 0 else Tj1
            Tj2 = 0 if Td == 0 else Tj2

    # 计算运动量
    #  aLima = jMax * Tj1
    #  aLimd = -jMax * Tj2
    #  vLim = max(v0 + (Ta - Tj1)*aLima, v1 - (Td - Tj2)*aLimd)
    #  print(f"Ta = {Ta}, Tv = {Tv}, Td = {Td}, Tj1 = {Tj1}, Tj2 = {Tj2}")
    #  print(f"vLim = {vLim}, aLima = {aLima}, aLimd = {aLimd}")

    return [Ta, Tv, Td, Tj1, Tj2, q0, q1, v0, v1, vMax, aMax, jMax]


def double_s_interpolate(param, t):
    """
    @brief 双S曲线插补，获取 t 时刻的运动学参数
    @param param [Ta, Tv, Td, Tj1, Tj2, q0, q1, v0, v1, vMax, aMax, jMax]
    """

    [Ta, Tv, Td, Tj1, Tj2, q0, q1, v0, v1, vMax, aMax, jMax] = param

    # 计算运动量
    aLima = jMax * Tj1
    aLimd = -jMax * Tj2
    vLim = max(v0 + (Ta - Tj1)*aLima, v1 - (Td - Tj2)*aLimd)
    #  print(f"vLim = {vLim}, aLima = {aLima}, aLimd = {aLimd}")

    Tavd = Ta + Tv + Td
    [curS, curV, curA, curJ] = [0,0,0,0]

    # 解决正负号
    if q1 < q0:
        q0, q1, v0, v1 = -q0, -q1, -v0, -v1
        vMax, aMax, jMax = -vMax, -aMax, -jMax

    # 加速阶段
    if t < Tj1:
        curS = q0 + v0*t + jMax*t*t*t/6
        curV = v0 + jMax*t*t/2
        curA = jMax*t
        curJ = jMax
    elif t < Ta - Tj1:
        curS = q0 + v0*t + aLima*(3*t*t - 3*Tj1*t + Tj1*Tj1)/6
        curV = v0 + aLima*(t-Tj1/2)
        curA = aLima
        curJ = 0
    elif t < Ta:
        curS = q0 + (vLim + v0)*Ta/2 - vLim*(Ta-t) + jMax*(Ta-t)**3/6
        curV = vLim - jMax*(Ta-t)*(Ta-t)/2
        curA = jMax*(Ta-t)
        curJ = -jMax
    # 匀速阶段
    elif t < Ta + Tv:
        curS = q0 + (vLim + v0)*Ta/2 + vLim*(t-Ta)
        curV = vLim
        curA = 0
        curJ = 0
    # 减速阶段
    elif t < Tavd - Td + Tj2:
        curS = q1 - (vLim + v1)*Td/2 + vLim*(t-Tavd+Td) - jMax*(t-Tavd+Td)**3/6
        curV = vLim - jMax*(t-Tavd+Td)**2/2
        curA = -jMax*(t-Tavd+Td)
        curJ = -jMax
    elif t < Tavd - Tj2:
        curS = q1 - (vLim + v1)*Td/2 + vLim*(t-Tavd+Td) + aLimd*(3*(t-Tavd+Td)**2 - 3*Tj2*(t-Tavd+Td) + Tj2*Tj2)/6
        curV = vLim + aLimd*(t-Tavd+Td-Tj2/2)
        curA = aLimd
        curJ = 0
    else:
        curS = q1 - v1*(Tavd-t) - jMax*(Tavd-t)**3/6
        curV = v1 + jMax*(Tavd-t)**2/2
        curA = -jMax*(Tavd-t)
        curJ = jMax

    return [curS, curV, curA, curJ]


def calc_fifth_polynomial_interp_parameters(q0, q1, v0, v1, T, a0 = 0, a1 = 0):
    """
    @brief 计算五次多项式插补的系数
           q(t) = q0 + k1*t + k2*t2 + k3*t3 + k4*t4 + k5*t5
    @param [q0, q1] 起止位置
    @param [v0, v1] 起止速度
    @param [a0, a1] 起止加速度
    @param T 运动时间
    """
    h = q1 - q0
    # 五次插值多项式系数
    k0 = q0
    k1 = v0
    k2 = a0/2
    k3 = (20*h - (8*v1+12*v0)*T - (3*a0-a1)*T*T) / (2*T*T*T)
    k4 = (-30*h + (14*v1+16*v0)*T + (3*a0-2*a1)*T*T) / (2*T*T*T*T)
    k5 = (12*h - 6*(v1+v0)*T + (a1-a0)*T*T) / (2*T*T*T*T*T)

    return [k0, k1, k2, k3, k4, k5]


def fifth_polynomial_interpolate(param, t):
    """
    @brief 五次多项式插补，获取 t 时刻的运动学参数
    @param param = [k0, k1, k2, k3, k4, k5]
    """
    [k0, k1, k2, k3, k4, k5] = param
    [curS, curV, curA, curJ] = [0,0,0,0]

    curS = k0 + k1*t + k2*t*t + k3*t*t*t + k4*t*t*t*t + k5*t*t*t*t*t
    curV = k1 + 2*k2*t + 3*k3*t*t + 4*k4*t*t*t + 5*k5*t*t*t*t
    curA = 2*k2 + 6*k3*t + 12*k4*t*t + 20*k5*t*t*t
    curJ = 6*k3 + 24*k4*t + 60*k5*t*t

    return [curS, curV, curA, curJ]


def test_multi_axis():
    # 两关节同步
    q01 = [0, 0, 0]
    q11 = [10, 12, 20]
    v01 = [1, 10, 4]
    v11 = [0, 0, 0]
    vMax, aMax, jMax = 10, 10, 30
    jntNum = len(q01)
    dt = 0.001

    # 一个关节用双S曲线插补，其余关节用五次多项式插补
    paramDoubleS = []
    paramFifthPoly = np.zeros([jntNum,6])

    t1 = np.empty([0])
    tMax, slowestJnt = 0, -1
    for i in range(jntNum):
        param = calc_double_s_interp_parameters(q01[i], q11[i], v01[i], v11[i], vMax, aMax, jMax)
        curTime = param[0] + param[1] + param[2]
        # 找到最慢的关节轴
        if curTime > tMax:
            tMax, slowestJnt = curTime, i
            paramDoubleS = param
        t1 = np.append(t1, param[0] + param[1] + param[2])
    #  print(t1)
    #  print(slowestJnt, ", ", tMax)

    # 计算插补参数
    for i in range(jntNum):
        if i != slowestJnt:
            paramFifthPoly[i] = calc_fifth_polynomial_interp_parameters(q01[i], q11[i], v01[i], v11[i], tMax)
    #  print(paramFifthPoly)

    #  开始插补
    tList = np.linspace(0, tMax, int(tMax/dt))
    sList, vList, aList, jList = [np.zeros([jntNum, len(tList)]) for i in range(4)]
    for tIdx in range(len(tList)):
        for i in range(jntNum):
            # 最慢的关节轴用双S曲线插补
            if i == slowestJnt:
                tmp = double_s_interpolate(paramDoubleS, tList[tIdx])
                [sList[i, tIdx], vList[i, tIdx], aList[i, tIdx], jList[i, tIdx]] = tmp
            # 其余关节按五次多项式插补
            else:
                tmp = fifth_polynomial_interpolate(paramFifthPoly[i], tList[tIdx])
                [sList[i, tIdx], vList[i, tIdx], aList[i, tIdx], jList[i, tIdx]] = tmp

    fig = plt.figure('plotUtils test',figsize=(16,9))
    for i in range(jntNum):
        #statements
        axS = plt.subplot(jntNum, 4, i*4+1)
        axV = plt.subplot(jntNum, 4, i*4+2)
        axA = plt.subplot(jntNum, 4, i*4+3)
        axJ = plt.subplot(jntNum, 4, i*4+4)
        axS.plot(tList, sList[i,:])
        axV.plot(tList, vList[i,:])
        axA.plot(tList, aList[i,:])
        axJ.plot(tList, jList[i,:])

    plt.show()


def test_single_axis():
    param = calc_double_s_interp_parameters(0, 10, 1, 0, 10, 10, 30)
    #  param = calc_double_s_interp_parameters(0, 10, 10, 0, 20, 40, 240)
    T = param[0] + param[1] + param[2]

    #  param = calc_fifth_polynomial_interp_parameters(0, 10, -5, -10, 8)
    #  T = 8

    dt = 0.001
    tList = np.linspace(0, T, int(T/dt))
    sList, vList, aList, jList = [np.array([]) for i in range(4)]

    for t in tList:
        tmp = double_s_interpolate(param, t)
        #  tmp = fifth_polynomial_interpolate(param, t)
        sList = np.append(sList, tmp[0])
        vList = np.append(vList, tmp[1])
        aList = np.append(aList, tmp[2])
        jList = np.append(jList, tmp[3])

    fig = plt.figure('plotUtils test',figsize=(8,6))
    axS =plt.subplot(411)
    axV =plt.subplot(412)
    axA =plt.subplot(413)
    axJ =plt.subplot(414)

    axS.plot(tList, sList)
    axV.plot(tList, vList)
    axA.plot(tList, aList)
    axJ.plot(tList, jList)

    plt.show()


if __name__ == "__main__":
    test_multi_axis()
    #  test_single_axis()



