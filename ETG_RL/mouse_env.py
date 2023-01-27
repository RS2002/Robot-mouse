from mujoco_py import load_model_from_path, MjSim,MjViewer
import numpy as np
from LegModel.forPath import LegPath
from LegModel.foreLeg import ForeLegM
from LegModel.hindLeg import HindLegM

class PPO_SimModel(object):
    def __init__(self, modelPath,T,dt,H,sigma_sq,phase,amp,T2_radio,max_steps=20000, view=False):
        super(PPO_SimModel, self).__init__()
        self.model = load_model_from_path(modelPath)  # 加载模型
        self.sim = MjSim(
            self.model)  # 状态模拟函数：其中默认参数包括nsubsteps=1，表示调用step函数时可运行步骤的数量为1，每次缓冲区交换1步； udd_callback=None，用户定义的回调，在step执行后使用udd_state函数
        '''self.viewer = MjViewer(self.sim)  # 3D渲染当前的模拟状态
        self.viewer.cam.azimuth = 0
        self.viewer.cam.lookat[0] += 0.25
        self.viewer.cam.lookat[1] += -0.5
        self.viewer.cam.distance = self.model.stat.extent * 0.5'''
        self.sim_state = self.sim.get_state()  # 返回模拟器状态的副本
        self.sim.set_state(self.sim_state)  # 设置模拟器状态
        self.legPosName = [
            # shoulder对应front left/right leg(fl/fr)，hip对应rear left/right leg(rl/rr)；router为上方关节、foot为下方关节
            ["router_shoulder_fl", "foot_s_fl"],
            ["router_shoulder_fr", "foot_s_fr"],
            ["router_hip_rl", "foot_s_rl"],
            ["router_hip_rr", "foot_s_rr"]]
        self.fixPoint = "body_ss"  # "neck_ss"
        '''self.legRealPoint_x = [[], [], [], []]
        self.legRealPoint_y = [[], [], [], []]
        self.movePath = [[], [], []]'''
        self.index = {"m1_fl": 0, "m2_fl": 1, "m1_fr": 2, "m2_fr": 3, "m1_rl": 4, "m2_rl": 5, "m1_rr": 6, "m2_rr": 7,
                      "m1_tail": 8,
                      "neck": 9, "head": 10, "spine": 11, "fl_t1": 12, "fr_t1": 13, "rl_t1": 14, "rr_t1": 15,
                      "com_pos": 16, "com_quat": 19, "com_vel": 23, "imu_acc": 26, "imu_gyro": 29}  # 设置传感器的序号
        self.y=[0,0]
        self.max_steps=max_steps
        self.steps=0
        self.view=view
        if not self.view:
            self.viewer=None
        else:
            self.viewer = MjViewer(self.sim)  # 3D渲染当前的模拟状态
            self.viewer.cam.azimuth = 0
            self.viewer.cam.lookat[0] += 0.25
            self.viewer.cam.lookat[1] += -0.5
            self.viewer.cam.distance = self.model.stat.extent * 0.5
        #ETG
        self.ETG_W=None
        self.ETG_b=None
        self.dt = dt
        self.T = T
        self.t = 0
        self.H = H
        self.sigma_sq = sigma_sq
        self.phase = phase
        self.amp = amp
        self.u = []
        self.omega = 2.0 * np.pi / T
        self.T2_ratio = T2_radio
        for h in range(H):
            t_now = h * self.T / (H - 0.9)
            self.u.append(self.forward(t_now))
        self.u = np.asarray(self.u).reshape(-1, 2)
        self.TD = 0
        self.time_step=0.01
        fl_params = {'lr0': 0.033, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.0295,
                     'l2': 0.0145, 'l3': 0.0225, 'l4': 0.0145, 'alpha': 23 * np.pi / 180}
        self.fl_left = ForeLegM(fl_params)
        self.fl_right = ForeLegM(fl_params)
        hl_params = {'lr0': 0.032, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.0317,
                     'l2': 0.02, 'l3': 0.0305, 'l4': 0.0205, 'alpha': 73 * np.pi / 180}
        self.hl_left = HindLegM(hl_params)
        self.hl_right = HindLegM(hl_params)

    def step(self, ctrlData,donef=False):
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
        if self.ETG_b is not None and self.ETG_W is not None:
            state = self.update2(t=self.steps*self.time_step)
            ctrl_add=self.ETG_forward(state)
            #print("ctrl_add",ctrl_add)
            ctrlData= np.asarray(ctrlData).reshape(-1)+ctrl_add

        self.steps+=1
        self.sim.data.ctrl[:] = ctrlData
        self.sim.step()  # 推进模拟
        #self.viewer.render()  # 将当前模拟状态显示在屏幕上
        state,pos=self.get_sensors()
        self.y[0]=self.y[1]
        self.y[1]=pos[1]
        reward=-(self.y[1]-self.y[0])
        done=False
        #加入跌倒检测机制
        if pos[2] < 0.045:
            reward -= 0.1
            done=True

        donef=False

        if self.steps>=self.max_steps or donef:
            done=True
        if self.view:
            self.viewer.render()  # 将当前模拟状态显示在屏幕上
        info={}
        info['pos']=pos
        return state,reward,done,info

    def get_sensors(self): #得到观测值与质心位置坐标
        sensors = self.sim.data.sensordata
        pos = sensors[self.index["com_pos"]:self.index["com_pos"] + 3].copy()  # 位置
        linvel = sensors[self.index["com_vel"]:self.index["com_vel"] + 3].copy()  # 线速度
        # acc = sensors[self.index["imu_acc"]:self.index["imu_acc"] + 3].copy()  # 加速度
        # angvel = sensors[self.index["imu_gyro"]:self.index["imu_gyro"] + 3].copy()  # 角速度
        frame_quat=sensors[self.index["com_quat"]:self.index["com_quat"] + 4].copy()  # 方向信息
        joint_pos=sensors[:12].copy()
        state=np.concatenate([linvel,frame_quat,joint_pos])
        #state=sensors.copy()
        return state,pos

    def reset(self,ETG_W=None,ETG_b=None):
        #self.sim.set_state(self.sim_state)
        self.sim.reset() #恢复初始状态
        self.steps=0
        state,pos=self.get_sensors()
        self.y[0]=pos[1]
        self.y[1]=pos[1]
        self.ETG_W=ETG_W
        self.ETG_b=ETG_b
        return state

    def forward(self, t):
        x = []
        for i in range(self.phase.shape[0]):
            x.append(self.amp * np.sin(self.phase[i] + t * self.omega))
        return np.asarray(x).reshape(-1)

    def update(self, t=None):
        time = t if t is not None else self.t
        x = self.forward(time)
        self.t += self.dt
        r = []
        for i in range(self.H):
            dist = np.sum(np.power(x - self.u[i], 2)) / self.sigma_sq
            r.append(np.exp(-dist))
        r = np.asarray(r).reshape(-1)
        return r

    def update2(self, t=None):
        time = t if t is not None else self.t
        x = self.forward(time)
        x2 = self.forward(time + self.T2_ratio * self.T)
        self.t += self.dt
        r = []
        for i in range(self.H):
            dist = np.sum(np.power(x - self.u[i], 2)) / self.sigma_sq
            r.append(np.exp(-dist))
        r = np.asarray(r).reshape(-1)
        r2 = []
        for i in range(self.H):
            dist = np.sum(np.power(x2 - self.u[i], 2)) / self.sigma_sq
            r2.append(np.exp(-dist))
        r2 = np.asarray(r2).reshape(-1)
        return (r, r2)

    def ETG_forward(self,x):
        x1 = np.asarray(x[0]).reshape(-1, 1)
        x2 = np.asarray(x[1]).reshape(-1, 1)
        act1 = self.ETG_W.dot(x1).reshape(-1) + self.ETG_b
        act2 = self.ETG_W.dot(x2).reshape(-1) + self.ETG_b
        act=np.zeros(12)
        act[:2] = self.fl_left.pos_2_angle(act1[0],act1[1])
        act[2:4] = self.fl_right.pos_2_angle(act2[0],act2[1])
        act[4:6] = self.hl_left.pos_2_angle(act2[0],act2[1])
        act[6:8] = self.hl_right.pos_2_angle(act1[0],act1[1])
        return act