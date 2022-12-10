import math

class HindLegM(object):
	"""docstring for HindLegM"""
	def __init__(self, leg_params):
		super(HindLegM, self).__init__()
		self.lr0 = leg_params['lr0']
		self.rp = leg_params['rp']
		self.d1 = leg_params['d1']
		
		self.l1 = leg_params['l1']
		self.l2 = leg_params['l2']
		self.l3 = leg_params['l3']
		self.l4 = leg_params['l4']

		self.alpha = leg_params['alpha']

		self.l_23 = self.l3 - self.l2
		self.l_a = self.LawOfCosines_edge(
			self.l3, self.l4, (math.pi-self.alpha))
		self.beta = self.LawOfCosines_angle(
			self.l_a, self.l3, self.l4)

		self.PI = math.pi
	def pos_2_angle(self, x, y):
		lt = math.sqrt(x*x + y*y)
		theta_2_t = self.LawOfCosines_angle(self.l1, self.l_a, lt)
		theta_2 = theta_2_t - self.beta
		#theta_1_t1 = self.LawOfCosines_angle(self.l1, lt, self.l_a)
		#theta_1_t2 = theta_1 - theta_1_t1

		theta_1_t1 = self.LawOfCosines_angle(self.l1, lt, self.l_a)
		theta_1_t2 = 0
		if x == 0:
			if y < 0:
				theta_1_t2 = 0
			else:
				theta_1_t2 = -self.PI
		else:
			theta_1_t2 = math.acos(-y/lt)*(x/abs(x))
		theta_1 = theta_1_t1 - theta_1_t2

		xB = self.l1*math.sin(theta_1) + self.l2*math.sin(theta_2-theta_1)
		yB = -self.l1*math.cos(theta_1) + self.l2*math.cos(theta_2-theta_1)

		OB = math.sqrt(xB*xB + yB*yB)

		q1 = theta_1
		q2 = (OB-self.lr0)/self.rp

		return [-q1, -q2]


	def LawOfCosines_edge(self, la, lb, angle_ab):
		lc_2 = la*la + lb*lb - 2*la*lb*math.cos(angle_ab)
		lc = math.sqrt(lc_2)
		return lc
	def LawOfCosines_angle(self, la, lb, lc):
		angle_ab_cos = (la*la + lb*lb - lc*lc)/(2*la*lb)
		#print("----> ", angle_ab_cos)
		angle_ab = math.acos(angle_ab_cos)
		return angle_ab
	