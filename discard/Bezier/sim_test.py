import argparse
from ToSim import SimModel
from Controller import MouseController
import time

RUN_STEPS = 5000
if __name__ == '__main__':
	parser = argparse.ArgumentParser("Description.")
	parser.add_argument('--fre', default=0.67,
		type=float, help="Gait stride") #设置步幅

	args = parser.parse_args()

	# theMouse = SimModel("../../models/dynamic_4l_t3.xml")
	theMouse = SimModel("../../models/scene_test3.xml")


	theController = MouseController(args.fre)
	for i in range(100):
		ctrlData = [0.0, 1.5, 0.0, 1.5, 0.0, -1.2, 0.0,-1.2, 0,0,0,0]
		theMouse.runStep(ctrlData)
	theMouse.initializing()
	start = time.time()
	for i in range(RUN_STEPS):
		tCtrlData = theController.runStep()				# No Spine
		#tCtrlData = theController.runStep_spine()		# With Spine
		ctrlData = tCtrlData
		theMouse.runStep(ctrlData)
	end = time.time()
	timeCost = end-start
	print("Time -> ", timeCost)
	print(theMouse.pos[0])
	print(theMouse.pos[-1])
	# theMouse.write_log()
	theMouse.close_window()
	# theMouse.drawPath()
