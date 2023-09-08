from mujoco_py import load_model_from_path, MjSim, MjViewer
import numpy as np
import math

import time
class SimModel(object):
	"""docstring for SimModel"""
	def __init__(self, modelPath):
		super(SimModel, self).__init__()
		self.model = load_model_from_path(modelPath) #加载模型
		self.sim = MjSim(self.model) #状态模拟函数：其中默认参数包括nsubsteps=1，表示调用step函数时可运行步骤的数量为1，每次缓冲区交换1步； udd_callback=None，用户定义的回调，在step执行后使用udd_state函数
		self.viewer = MjViewer(self.sim) #3D渲染当前的模拟状态
		self.viewer.cam.azimuth = 0
		self.viewer.cam.lookat[0] += 0.25
		self.viewer.cam.lookat[1] += -0.5
		self.viewer.cam.distance = self.model.stat.extent * 0.5

		self.sim_state = self.sim.get_state() #返回模拟器状态的副本
		self.sim.set_state(self.sim_state) #设置模拟器状态
		self.legPosName = [   #shoulder对应front left/right leg(fl/fr)，hip对应rear left/right leg(rl/rr)；router为上方关节、foot为下方关节
			["router_shoulder_fl", "foot_s_fl"],
			["router_shoulder_fr", "foot_s_fr"],
			["router_hip_rl", "foot_s_rl"],
			["router_hip_rr", "foot_s_rr"]]
		self.fixPoint = "body_ss"#"neck_ss"
		self.legRealPoint_x = [[],[],[],[]]
		self.legRealPoint_y = [[],[],[],[]]
		self.movePath = [[],[],[]]
		self.index={"m1_fl":0,"m2_fl":1,"m1_fr":2,"m2_fr":3,"m1_rl":4,"m2_rl":5,"m1_rr":6,"m2_rr":7,"m1_tail":8,
					"neck":9,"head":10,"spine":11,"fl_t1":12,"fr_t1":13,"rl_t1":14,"rr_t1":15,
					"com_pos":16,"com_quat":19,"com_vel":23,"imu_acc":26,"imu_gyro":29}#设置传感器的序号
		#存储运动过程的位置、线速度、角速度、加速度
		self.pos = []
		self.linvel = []
		self.angvel = []
		self.acc = []
		#存储受力传感器数据(位于foot)
		self.fl_t1 = [] #前左
		self.fr_t1 = [] #前右
		self.rl_t1 = [] #后左
		self.rr_t1 = [] #后右
		self.digit = 4 #传感器数据保留小数点位数


	def initializing(self):
		self.movePath = [[],[],[]]
		self.legRealPoint_x = [[],[],[],[]]
		self.legRealPoint_y = [[],[],[],[]]
		self.pos = []
		self.linvel = []
		self.angvel = []
		self.acc = []
		self.fl_t1 = []
		self.fr_t1 = []
		self.rl_t1 = []
		self.rr_t1 = []


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

		leg_point_x=[]
		leg_point_y=[]

		self.sim.data.ctrl[:] = ctrlData
		self.sim.step() #推进模拟
		self.viewer.render() #将当前模拟状态显示在屏幕上

		tData = self.sim.data.get_site_xpos(self.fixPoint)
		for i in range(3):
			self.movePath[i].append(tData[i])
		for i in range(4):
			originPoint = self.sim.data.get_site_xpos(self.legPosName[i][0])
			currentPoint = self.sim.data.get_site_xpos(self.legPosName[i][1])
			#print(originPoint, currentPoint)
			tX = currentPoint[1]-originPoint[1]
			tY = currentPoint[2]-originPoint[2]
			self.legRealPoint_x[i].append(tX)
			self.legRealPoint_y[i].append(tY)
			leg_point_x.append(tX)
			leg_point_y.append(tY)

		result,result2=self.get_sensors()
		self.pos.append(result['pos'])
		self.linvel.append(result['linvel'])
		self.angvel.append(result['angvel'])
		self.acc.append(result['acc'])
		self.fl_t1.append(result2['fl_t1'])
		self.fr_t1.append(result2['fr_t1'])
		self.rl_t1.append(result2['rl_t1'])
		self.rr_t1.append(result2['rr_t1'])

		return leg_point_x,leg_point_y


	def getTime(self):
		return self.sim.data.time

	def point_distance_line(self, point,line_point1,line_point2): #计算point到直线line_point1->line_point2的距离
		vec1 = line_point1 - point
		vec2 = line_point2 - point
		distance = np.linalg.norm(np.cross(vec1,vec2)) / np.linalg.norm(line_point1-line_point2)
		return distance

	def drawPath(self):
		import matplotlib.pyplot as plt
		ax = plt.axes(projection='3d')
		ax.plot3D(self.movePath[0], self.movePath[1], self.movePath[2])
		plt.show()
		for i in range(4):
			plt.plot(self.legRealPoint_x[i], self.legRealPoint_y[i])
			plt.show()
		distance = 0  # 计算走过的路径长度
		for i in range(len(self.movePath[0])):
			point = np.array([self.movePath[0][i], self.movePath[0][i], self.movePath[0][i]])
			if i == 0:
				line_point1 = np.array([0, 0, 0])
				line_point2 = np.array([0, 0, 1])
			else:
				line_point1 = np.array([self.movePath[0][i - 1], self.movePath[0][i - 1], self.movePath[0][i - 1]])
				line_point2 = np.array([self.movePath[0][i - 1], self.movePath[0][i - 1], self.movePath[0][i - 1] + 1])
			distance += self.point_distance_line(point, line_point1, line_point2)  # 以相邻两次位置相邻在xoy平面上的投影作为移动距离
			#print(distance)
		return distance

	def get_sensors(self):
		sensors = self.sim.data.sensordata
		pos = sensors[self.index["com_pos"]:self.index["com_pos"] + 3].copy()  # 位置
		linvel = sensors[self.index["com_vel"]:self.index["com_vel"] + 3].copy()  # 线速度
		acc = sensors[self.index["imu_acc"]:self.index["imu_acc"] + 3].copy()  # 加速度
		angvel = sensors[self.index["imu_gyro"]:self.index["imu_gyro"] + 3].copy()  # 角速度
		result = {'pos': pos, 'linvel': linvel, 'acc': acc, 'angvel': angvel}
		fl_t1 = sensors[self.index["fl_t1"]]
		fr_t1 = sensors[self.index["fr_t1"]]
		rl_t1 = sensors[self.index["rl_t1"]]
		rr_t1 = sensors[self.index["rr_t1"]]
		result2 = {'fl_t1': fl_t1, 'fr_t1': fr_t1, 'rl_t1': rl_t1, 'rr_t1': rr_t1}
		return result,result2

	def write_log(self):  # 记录传感器数据
		f1 = open("position.txt", mode='w')
		f2 = open("linear velocity.txt", mode='w')
		f3 = open("angular velocity.txt", mode='w')
		f4 = open("acceleration.txt", mode='w')
		f5 = open("log.txt", mode='w')
		f6 = open("touch_fl.txt", mode='w')
		f7 = open("touch_fr.txt", mode='w')
		f8 = open("touch_rl.txt", mode='w')
		f9 = open("touch_rr.txt", mode='w')
		for i in range(len(self.pos)):
			pos_scalar = self.compute_L2(self.pos[i])
			linvel_scalar = self.compute_L2(self.linvel[i])
			angvel_scalar = self.compute_L2(self.angvel[i])
			acc_scalar = self.compute_L2(self.acc[i])
			f1.writelines(str(self.pos[i]) + ":  " + str(pos_scalar) + "\n")
			f2.writelines(str(self.linvel[i]) + ":  " + str(linvel_scalar) + "\n")
			f3.writelines(str(self.angvel[i]) + ":  " + str(angvel_scalar) + "\n")
			f4.writelines(str(self.acc[i]) + ":  " + str(acc_scalar) + "\n")
			f5.writelines("位置" + str(self.pos[i]) + ": " + str(pos_scalar))
			f5.writelines(" 线速度" + str(self.linvel[i]) + ": " + str(linvel_scalar))
			f5.writelines(" 角速度" + str(self.angvel[i]) + ": " + str(angvel_scalar))
			f5.writelines(" 加速度" + str(self.acc[i]) + ": " + str(acc_scalar) + "\n")
			f6.writelines(str(self.fl_t1[i]) + "\n")
			f7.writelines(str(self.fr_t1[i]) + "\n")
			f8.writelines(str(self.rl_t1[i]) + "\n")
			f9.writelines(str(self.rr_t1[i]) + "\n")
		f1.close()
		f2.close()
		f3.close()
		f4.close()
		f5.close()
		f6.close()
		f7.close()
		f8.close()
		f9.close()

	def compute_L2(self,arr):
		sum=0
		for i in range(len(arr)):
			sum+=arr[i]**2
			arr[i]=round(arr[i],self.digit)
		sum=round(math.sqrt(sum),self.digit)
		return sum



