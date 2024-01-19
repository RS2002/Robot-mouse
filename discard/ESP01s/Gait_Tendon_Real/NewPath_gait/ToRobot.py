import socket
import math
import time
import sys

'''
ID:
0  --> Head
1  --> Fore-Left Leg		2 --> Fore-Left Coil
3  --> Fore-Right Leg		4 --> Fore-Right Coil
5  --> Hind-Left Leg		6 --> Hind-Left Coil
7  --> Hind-Right Leg		8 --> Hind-Right Coil
9  --> Spine
10 --> Tail

Unit: degree [-90, 90]
'''

class RobotDriver(object):
	"""docstring for RobotDriver"""
	MotorNum = 11
	TimeStep = 0.05 # Unit -> Second
	def __init__(self, robot_ip, robot_port, timeStep=0.05):
		super(RobotDriver, self).__init__()
		# For Connection
		#self.s_address = ('192.168.12.69', 6666)
		self.TimeStep = timeStep
		self.s_address = (robot_ip, robot_port)
		self.theRobot = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		# For control
		"""
		Transfer:
		FL Leg : * +1 		FL Coil: * -1  -> FL Coil: * -1  FL Leg : * 1
		FR Leg : * -1 		FR Coil: * +1
		HL Leg : * +1 		HL Coil: * -1
		HR Leg : * -1 		HR Coil: * +1
		Spine  : * -1
		"""
		self.sim2real = {
			"leg":[1, -1, -1, 1, 1, -1, -1, 1],
			"spine": -1
		}
		self.angleList = [0]*self.MotorNum
		self.ctrlData = [0]*self.MotorNum
		self.initAngle = [0,4,0,0,0,0,0,6,0,0,0]

	def runStep(self, leg_ctrl, spine_ctrl, head_ctrl, tail_ctrl, start_time):
		self.angleList[0] = head_ctrl
		#print(leg_ctrl)
		for i in range(8):
			self.angleList[i+1] = leg_ctrl[i] * self.sim2real["leg"][i]
		self.angleList[9] = 0#spine_ctrl*self.sim2real["spine"]
		self.angleList[10] = 0#tail_ctrl
		# Rad to degree 
		# From [-PI/2, PI/2] to [-90, 90]
		for i in range(self.MotorNum):
			self.ctrlData[i] = self.angleList[i] * 180 / math.pi
			self.ctrlData[i] += self.initAngle[i]

		#print(self.ctrlData)
		retrunData = self.toInteract(self.ctrlData)

		timeCost = time.time()-start_time
		if timeCost < self.TimeStep:
			time.sleep(self.TimeStep-timeCost)

	def toInteract(self, sendData):
		theString = ""
		for i in range(self.MotorNum):
			tStr = str(int(sendData[i]))
			theString = theString + tStr + ","
		self.theRobot.sendto(theString.encode(), self.s_address)
		data_r = []
		for i in range(10):
			try:
				self.theRobot.settimeout(0.02)
				data_r, addr = self.theRobot.recvfrom(1024)  # Receive feedback from robot
				data_r = data_r.decode()
				break
			except socket.timeout:
				print("UDP Time out --> ", i)
				self.theRobot.sendto(theString.encode(), self.s_address)
		return data_r

	def shutdown(self):
		self.theRobot.close()