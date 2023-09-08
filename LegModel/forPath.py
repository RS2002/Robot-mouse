import math
import numpy as np

class LegPath(object):
	"""docstring for ForeLegPath"""
	def __init__(self, pathType="circle"):
		super(LegPath, self).__init__()
		# Trot #0维表示初始位置坐标，1维为椭圆运动轨迹的两个参数
		#F为前腿、H为后腿，U表示抬腿、D表示放下腿
		self.para_FU = [[-0.00, -0.045], [0.03, 0.01]]
		self.para_FD = [[-0.00, -0.045], [0.03, 0.005]]
		self.para_HU = [[0.00, -0.05], [0.03, 0.01]]
		self.para_HD = [[0.00, -0.05], [0.03, 0.005]]

	def getOvalPathPoint(self, radian, leg_flag, halfPeriod):
		pathParameter = None
		cur_radian = 0
		if leg_flag == "F":
			if radian < halfPeriod*math.pi:
				pathParameter = self.para_FU
				cur_radian = radian/halfPeriod
			else:
				pathParameter = self.para_FD
				cur_radian = (radian)/(2-halfPeriod)
		else:
			if radian < halfPeriod*math.pi:
				pathParameter = self.para_HU
				cur_radian = radian/halfPeriod
			else:
				pathParameter = self.para_HD
				cur_radian = (radian)/(2-halfPeriod)
		originPoint = pathParameter[0]
		ovalRadius = pathParameter[1]
		#根据椭圆方程和角度求坐标位置
		trg_x = originPoint[0] + ovalRadius[0] *math.cos(cur_radian)
		trg_y = originPoint[1] + ovalRadius[1] *math.sin(cur_radian)
		return [trg_x, trg_y]

	def getBezierPathPoint(self, radian, leg_flag, halfPeriod):

		def get_Bezier_point(tao, delta, control_point_list, St, dx=0.0, dy=0.0):
			'''control_point_list[:, 0] += dx
            control_point_list[:, 1] += dy'''
			St = St % 2
			n = len(control_point_list)
			if St >= 0 and St < 1:
				return tao * (1 - 2 * St) + dx, delta * math.cos(math.pi * (1 - 2 * St) / 2) + dy
			elif St >= 1 and St < 2:
				x, y = 0, 0
				for j in range(n):
					x += math.factorial(n - 1) / math.factorial(j) / math.factorial(n - 1 - j) * math.pow(
						(St - 1), j) * math.pow(1 - (St - 1), n - 1 - j) * control_point_list[j][0]
					y += math.factorial(n - 1) / math.factorial(j) / math.factorial(n - 1 - j) * math.pow(
						(St - 1), j) * math.pow(1 - (St - 1), n - 1 - j) * control_point_list[j][1]
				return x + dx, y + dy

		pathParameter = None
		cur_radian = 0
		if leg_flag == "F":
			if radian < halfPeriod * math.pi:
				pathParameter = self.para_FU
				cur_radian = radian / halfPeriod
			else:
				pathParameter = self.para_FD
				cur_radian = (radian) / (2 - halfPeriod)
		else:
			if radian < halfPeriod * math.pi:
				pathParameter = self.para_HU
				cur_radian = radian / halfPeriod
			else:
				pathParameter = self.para_HD
				cur_radian = (radian) / (2 - halfPeriod)
		originPoint = pathParameter[0]
		params = pathParameter[1]
		tao = params[0]  # 前进步长的一半
		psi = params[1]*3/2  # 地上的最高点
		delta = -params[1]*1/2  # 地下的最低点
		control_point_list = np.array([
			[-tao, 0],
			[-1.4 * tao, 0],
			[-1.5 * tao, 0.9 * psi],
			[-1.5 * tao, 0.9 * psi],
			[-1.5 * tao, 0.9 * psi],
			[0, 0.9 * psi],
			[0, 0.9 * psi],
			[0, 1.1 * psi],
			[1.5 * tao, 1.1 * psi],
			[1.5 * tao, 1.1 * psi],
			[1.4 * tao, 0],
			[tao, 0]
		])

		cur_radian=2-cur_radian/math.pi
		#print(cur_radian)
		return get_Bezier_point(tao, delta, control_point_list, cur_radian, dx=originPoint[0], dy=originPoint[1])

