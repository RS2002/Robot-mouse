from ToRobot import RobotDriver
from Controller import MouseController
import time

if __name__ == '__main__':
	timeStep = 0.05#5 # s
	#theMouse = RobotDriver('192.168.12.97', 6666, timeStep)
	theMouse = RobotDriver('10.1.1.103', 6666, timeStep)
	print("Robot Connected !!!")
	theController = MouseController(timeStep, 0.67)
	leg_ctrl = [0]*8
	spine_ctrl = 0
	head_ctrl = 0
	tail_ctrl = 0
	print("Controller initiated !!!")
	print("Step 1 -->")
	for i in range(10):
		start_time = time.time()
		head_ctrl = -0.5
		theMouse.runStep(leg_ctrl, spine_ctrl, head_ctrl, tail_ctrl, start_time)
	print("Step 2 -->")
	for i in range(10):
		start_time = time.time()
		head_ctrl = 0.5
		theMouse.runStep(leg_ctrl, spine_ctrl, head_ctrl, tail_ctrl, start_time)
	print("Step 3 -->")
	for i in range(10):
		start_time = time.time()
		head_ctrl = 0
		theMouse.runStep(leg_ctrl, spine_ctrl, head_ctrl, tail_ctrl, start_time)
	print("Robot initialized !!!")
	#"""
	for i in range(3000):
		start_time = time.time()
		leg_ctrl, spine_ctrl, head_ctrl, tail_ctrl = theController.runStep()
		theMouse.runStep(leg_ctrl, spine_ctrl, head_ctrl, tail_ctrl, start_time)
	#"""

	for i in range(10):
		start_time = time.time()
		head_ctrl = -0.5
		theMouse.runStep([0]*8, 0, 0, 0, start_time)
	theMouse.shutdown()



