import mujoco
import mujoco.viewer
import numpy as np
import math

class SimModel(object):
    """docstring for SimModel"""

    def __init__(self, modelPath, max_steps=5000, dis_limit=3, render=False, save_path=None):
        super(SimModel, self).__init__()
        self.model = mujoco.MjModel.from_xml_path(modelPath)  # 加载模型
        self.data = mujoco.MjData(self.model)
        self.render = render
        if self.render:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        else:
            self.viewer = None

        # media.show_image(renderer.render())
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

        self.max_steps = max_steps
        self.steps = 0
        # save ctrldata
        self.ctrldata = np.array([])
        self.save_path = save_path
        self.dis_limit = dis_limit

    def initializing(self):
        self.steps = 0
        self.ctrldata = np.array([])
        state, pos, vel = self.get_sensors()
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

        self.data.ctrl[:] = ctrlData
        mujoco.mj_step(self.model, self.data)
        self.steps+=1
        if self.render:
            self.viewer.sync()
        state,pos,vel = self.get_sensors()
        reward = self.get_reward(vel)
        done=self.is_done(pos)
        if self.save_path is not None:
            self.ctrldata = np.concatenate((self.ctrldata, ctrlData), axis=0)
            if done:
                np.savez(self.save_path, ctrldata=self.ctrldata)
        return state, reward, done, pos

    def close_window(self):
        if self.render:
            self.viewer.close()

    def getTime(self):
        return self.data.time

    def get_sensors(self):
        sensors = self.data.sensordata
        pos = sensors[self.index["com_pos"]:self.index["com_pos"] + 3].copy()  # 位置
        acc = sensors[self.index["imu_acc"]:self.index["imu_acc"] + 3].copy()  # 加速度
        angvel = sensors[self.index["imu_gyro"]:self.index["imu_gyro"] + 3].copy()  # 角速度
        framequat = sensors[self.index["com_quat"]:self.index["com_quat"] + 4].copy() # 四元数（代替Pitch Angle（俯仰角）和Body Roll（滚转角））
        state = np.concatenate([acc, angvel, framequat])
        linvel = sensors[self.index["com_vel"]:self.index["com_vel"] + 3].copy()  # 线速度
        return state,pos,linvel

    def get_reward(self,linvel):
        reward = -linvel[1] * 4
        return reward

    def is_done(self,pos):
        done=False
        if self.steps >= self.max_steps: # 步数检测
            done = True
        if math.sqrt(pos[0]**2 + pos[1]**2) >= self.dis_limit: # 距离检测
            done = True
        if pos[2] < 0.045: # 跌倒检测
            done=True
        return done

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        return self.initializing()

    def get_step(self):
        return self.steps
