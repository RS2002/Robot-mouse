import copy
import mujoco
import mujoco.viewer
import numpy as np
from Bezier import get_Bezier_point,pos_2_angle
import math
import random
import torch
import matplotlib.pyplot as plt

L_span = 0.03
delta_St = np.array([0, 1, 1, 0])
dt = 0.005
para = [[[-0.00, -0.045], [0.03, 0.01]], [[-0.00, -0.045], [0.03, 0.005]], [[0.00, -0.05], [0.03, 0.01]],[[0.00, -0.05], [0.03, 0.005]]]
max_dis=0.025

class Env(object):
    def __init__(self, modelPath, max_steps=5000, view=False):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path(modelPath)  # 加载模型
        self.data = mujoco.MjData(self.model)
        self.render = view
        if self.render:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        else:
            self.viewer = None

        self.legPosName = [
            # shoulder对应front left/right leg(fl/fr)，hip对应rear left/right leg(rl/rr)；router为上方关节、foot为下方关节
            ["router_shoulder_fl", "foot_s_fl"],
            ["router_shoulder_fr", "foot_s_fr"],
            ["router_hip_rl", "foot_s_rl"],
            ["router_hip_rr", "foot_s_rr"]]
        self.fixPoint = "body_ss"  # "neck_ss"
        self.index = {"m1_fl": 0, "m2_fl": 1, "m1_fr": 2, "m2_fr": 3, "m1_rl": 4, "m2_rl": 5, "m1_rr": 6, "m2_rr": 7,
                      "m1_tail": 8,
                      "neck": 9, "head": 10, "spine": 11, "fl_t1": 12, "fr_t1": 13, "rl_t1": 14, "rr_t1": 15,
                      "com_pos": 16, "com_quat": 19, "com_vel": 23, "imu_acc": 26, "imu_gyro": 29}  # 设置传感器的序号

        self.movePath = [[], [], []]

        self.last_pos = [0, 0, 0]
        self.last_action = 0
        self.max_steps = max_steps
        self.steps = 0
        self.ctrldata = np.array([])
        self.t=0
        self.old_ctrl=0
        self.n=0
        self.initializing()

    def initializing(self):
        #初始化
        for i in range(50):
            self.data.ctrl = [0.0, 1.5, 0.0, 1.5, 0.0, -1.2, 0.0, -1.2, 0, 0, 0, 0]
            mujoco.mj_step(self.model, self.data)
            if self.render:
                self.viewer.sync()
        # self.random_env()
        self.old_ctrl=0
        self.t=0
        self.steps = 0
        self.ctrldata = np.array([])
        state, pos, vel = self.get_sensors()
        self.last_pos = copy.deepcopy(pos)
        self.last_action = 0
        self.n=0
        self.movePath = [[], [], []]

        # quat=state[-4:]
        # rm=quat2rm(quat)
        state = np.concatenate([[-1]*2, state, self.t + delta_St, [0]])
        # state = np.concatenate([[-1]*10, state, self.t + delta_St, [0]])

        return state


    def runStep(self, ctrlData):
        # ------------------------------------------ #
        # ID 0, 1 left-fore leg and coil
        # ID 2, 3 right-fore leg and coil
        # ID 4, 5 left-hide leg and coil
        # ID 6, 7 right-hide leg and coil
        # Note: For leg, it has [-1: front; 1: back]
        # Note: For fore coil, it has [-1: leg up; 1: leg down]
        # Note: For hide coil, it has [-1: leg down; 1: leg up]
        # ------------------------------------------ #
        # ID 08 is neck		(Horizontal)
        # ID 09 is head		(vertical)
        # ID 10 is spine	(Horizontal)  [-1: right, 1: left]
        # Note: range is [-1, 1]
        # ------------------------------------------ #

        ctrlData*=max_dis
        ctrlData[2:]*=0.01

        # ctrlData=0.3*self.old_ctrl+0.7*ctrlData
        self.old_ctrl=copy.deepcopy(ctrlData)

        psi=ctrlData[0]+max_dis
        delta=ctrlData[1]-max_dis

        # psi = 0.01
        # delta = -0.005

        old=np.array(self.data.ctrl[:8])

        for i in range(4):
            x, y = get_Bezier_point(L_span, psi, delta, self.t + delta_St[i])
            if i < 2:
                leg = "f"
            else:
                leg = "h"

            dx = para[i][0][0]
            dy = para[i][0][1]

            X = x + dx
            Y = y + dy

            self.data.ctrl[i * 2:(i + 1) * 2] = pos_2_angle(X, Y, leg)
        # self.data.ctrl[:8]+=ctrlData[2:]
        self.data.ctrl[8:]=0

        now=np.array(self.data.ctrl[:8])
        dif=np.mean(np.abs(now-old))

        mujoco.mj_step(self.model, self.data)
        self.steps+=1
        if self.render:
            self.viewer.sync()

        tData = self.data.site(self.fixPoint).xpos
        for i in range(3):
            self.movePath[i].append(tData[i])

        state,pos,linvel = self.get_sensors()

        c_F=np.sum(state[:4])
        if c_F ==0:
            self.n+=1
        else:
            self.n=0

        # quat=state[-4:]
        # rm=quat2rm(quat)

        if abs(linvel[1])<0.35:
            reward = -linvel[1] * 4
        else:
            reward = - 0.1

        # reward-=0.1*dif

        # if self.n>10:
        #     reward-=0.1

        # if abs(linvel[0])>0.08:
        #     reward -= 0.1


        self.t = (self.t + dt) % 2
        state = np.concatenate([ctrlData, state, self.t + delta_St, [reward]])





        done=False
        if self.steps >= self.max_steps:
            done = True


        self.last_pos=copy.deepcopy(pos)
        return state, reward, done, pos

    def get_sensors(self):  # 得到观测值与质心位置坐标
        sensors = self.data.sensordata
        pos = sensors[self.index["com_pos"]:self.index["com_pos"] + 3].copy()  # 位置
        acc = sensors[self.index["imu_acc"]:self.index["imu_acc"] + 3].copy()  # 加速度
        angvel = sensors[self.index["imu_gyro"]:self.index["imu_gyro"] + 3].copy()  # 角速度
        framequat = sensors[self.index["com_quat"]:self.index["com_quat"] + 4].copy() # 四元数（代替Pitch Angle（俯仰角）和Body Roll（滚转角））
        linvel = sensors[self.index["com_vel"]:self.index["com_vel"] + 3].copy()  # 线速度
        c_F = sensors[self.index["fl_t1"]:self.index["fl_t1"] + 4].copy()  # 腿部压力传感器
        state = np.concatenate([c_F, linvel, angvel, framequat])
        return state,pos,linvel

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        return self.initializing()

    def random_env(self):
        # self.model.opt (可以修改如重力、风速之类的参数)
        # self.model.opt.gravity[-1]=-5
        # print(self.model.opt.gravity)
        # self.model.opt.gravity[-1] = -12 + random.random() * 5

        for i in range(1, 451):
            self.model.geom("box{}".format(i)).size[-1] = random.random() * 0.01
        return

    def drawPath(self):
        ax = plt.axes(projection='3d')
        ax.plot3D(self.movePath[0], self.movePath[1], self.movePath[2])
        ax.set_xlabel('X', fontsize=16)
        ax.set_ylabel('Y', fontsize=16)
        ax.set_zlabel('Z', fontsize=16)
        plt.show()
def generate_boxes(x_min=-0.5, x_max=0.5, y_min=-3.5, y_max=1, size=0.05):
    with open("box.txt", 'w') as file:
        x = x_min
        counter = 1
        while x_max - x > 1e-6:
            y = y_min
            while y_max - y > 1e-6:
                file.write(
                    '<geom name="box{}" type="box" size="{:.3} {:.3} {:.3}" pos="{:.3} {:.3} 0" rgba="0.5 0.5 0.5 1"/> \n'.format(
                        counter, size, size, size, x, y))
                y += size * 2
                counter += 1
            x += size * 2


def quat2rm(q):
    r=np.zeros([3,3])
    r[0,0]=q[0]**2+q[1]**2-q[2]**2-q[3]**2
    r[0,1]=2*(q[1]*q[2]-q[0]*q[3])
    r[0,2]=2*(q[1]*q[3]+q[0]*q[2])
    r[1,0]=2*(q[1]*q[2]+q[0]*q[3])
    r[1,1]=q[0]**2-q[1]**2+q[2]**2-q[3]**2
    r[1,2]=2*(q[2]*q[3]-q[0]*q[1])
    r[2,0]=2*(q[1]*q[3]-q[0]*q[2])
    r[2,1]=2*(q[2]*q[3]+q[0]*q[1])
    r[2,2]=q[0]**2-q[1]**2-q[2]**2+q[3]**2
    return r

if __name__ == '__main__':
    generate_boxes()