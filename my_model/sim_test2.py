import argparse
from ToSim import SimModel
from Controller2 import Controller
import time

RUN_STEPS = 10000


def control_1(theMouse):
    print("control1")
    ctrlData = [0.0, 1.5, 0.0, 1.5, 0.0, -1.2, 0.0, -1.2, 0, 0, 0, 0]
    for i in range(10):
        ctrlData = [0.0, 1.5, 0.0, 1.5, 0.0, -1.2, 0.0, -1.2, 0, 0, 0, 0]
        leg_point_x, leg_point_y = theMouse.runStep(ctrlData)
    NUM = 100

    '''for i in range(NUM):
		foreLeg_left_q = theController.get_leg_ctrl(0, leg_point_x[0] , leg_point_y[0] + 0.025 / NUM * i)
		hindLeg_left_q = theController.get_leg_ctrl(2, leg_point_x[2], leg_point_y[2] + 0.025 / NUM * i)
		ctrlData[0] = foreLeg_left_q[0]
		ctrlData[1] = foreLeg_left_q[1]
		ctrlData[4] = hindLeg_left_q[0]
		ctrlData[5] = hindLeg_left_q[1]
		theMouse.runStep(ctrlData)

	for i in range(NUM):
		foreLeg_right_q = theController.get_leg_ctrl(1, leg_point_x[1] - 0.0125 / NUM * i,
													 leg_point_y[1] + 0.025 / NUM * i)
		ctrlData[2] = foreLeg_right_q[0]
		ctrlData[3] = foreLeg_right_q[1]
		theMouse.runStep(ctrlData)

	for i in range(NUM):
		foreLeg_left_q = theController.get_leg_ctrl(0, leg_point_x[0] ,
													leg_point_y[0] + 0.025 - 0.025 / NUM * i)
		hindLeg_left_q = theController.get_leg_ctrl(2, leg_point_x[2], leg_point_y[2] + 0.025 - 0.025 / NUM * i)
		foreLeg_right_q = theController.get_leg_ctrl(1, leg_point_x[1]- 0.0125 ,
													 leg_point_y[1] + 0.025 - 0.025 / NUM * i)
		ctrlData[0] = foreLeg_left_q[0]
		ctrlData[1] = foreLeg_left_q[1]
		ctrlData[4] = hindLeg_left_q[0]
		ctrlData[5] = hindLeg_left_q[1]
		ctrlData[2] = foreLeg_right_q[0]
		ctrlData[3] = foreLeg_right_q[1]
		theMouse.runStep(ctrlData)

	for i in range(NUM):
		foreLeg_right_q = theController.get_leg_ctrl(1, leg_point_x[1]- 0.0125, leg_point_y[1] + 0.025 / NUM * i)
		hindLeg_right_q = theController.get_leg_ctrl(3, leg_point_x[3], leg_point_y[3] + 0.025 / NUM * i)
		ctrlData[2] = foreLeg_right_q[0]
		ctrlData[3] = foreLeg_right_q[1]
		ctrlData[6] = hindLeg_right_q[0]
		ctrlData[7] = hindLeg_right_q[1]
		theMouse.runStep(ctrlData)

	for i in range(NUM):
		foreLeg_left_q = theController.get_leg_ctrl(0, leg_point_x[0] - 0.0125 / NUM * i,
													leg_point_y[0] + 0.025 / NUM * i)
		ctrlData[0] = foreLeg_left_q[0]
		ctrlData[1] = foreLeg_left_q[1]
		theMouse.runStep(ctrlData)

	for i in range(NUM):
		foreLeg_right_q = theController.get_leg_ctrl(1, leg_point_x[1]- 0.0125, leg_point_y[1] + 0.025 - 0.025 / NUM * i)
		hindLeg_right_q = theController.get_leg_ctrl(3, leg_point_x[3], leg_point_y[3] + 0.025 - 0.025 / NUM * i)
		foreLeg_left_q = theController.get_leg_ctrl(0, leg_point_x[0] - 0.0125,
													leg_point_y[0] + 0.025 - 0.025 / NUM * i)
		ctrlData[2] = foreLeg_right_q[0]
		ctrlData[3] = foreLeg_right_q[1]
		ctrlData[0] = foreLeg_left_q[0]
		ctrlData[1] = foreLeg_left_q[1]
		ctrlData[6] = hindLeg_right_q[0]
		ctrlData[7] = hindLeg_right_q[1]
		theMouse.runStep(ctrlData)

	ctrlData = [0.0, 1.5, 0.0, 1.5, 0.0, -1.2, 0.0, -1.2, 0, 0, 0, 0]
	leg_point_x, leg_point_y = theMouse.runStep(ctrlData)'''

    for i in range(NUM):
        foreLeg_right_q = theController.get_leg_ctrl(1, leg_point_x[1], leg_point_y[1] + 0.025 / NUM * i)
        hindLeg_right_q = theController.get_leg_ctrl(3, leg_point_x[3], leg_point_y[3] + 0.025 / NUM * i)
        ctrlData[2] = foreLeg_right_q[0]
        ctrlData[3] = foreLeg_right_q[1]
        ctrlData[6] = hindLeg_right_q[0]
        ctrlData[7] = hindLeg_right_q[1]
        theMouse.runStep(ctrlData)

    for i in range(NUM):
        hindLeg_left_q = theController.get_leg_ctrl(2, leg_point_x[2] - 0.05 / NUM * i, leg_point_y[2] + 0.05 / NUM * i)
        ctrlData[4] = hindLeg_left_q[0]
        ctrlData[5] = hindLeg_left_q[1]
        theMouse.runStep(ctrlData)

    for i in range(NUM):
        foreLeg_right_q = theController.get_leg_ctrl(1, leg_point_x[1], leg_point_y[1] + 0.025 - 0.025 / NUM * i)
        hindLeg_right_q = theController.get_leg_ctrl(3, leg_point_x[3], leg_point_y[3] + 0.025 - 0.025 / NUM * i)
        hindLeg_left_q = theController.get_leg_ctrl(2, leg_point_x[2] - 0.05, leg_point_y[2] + 0.05 - 0.05 / NUM * i)
        ctrlData[2] = foreLeg_right_q[0]
        ctrlData[3] = foreLeg_right_q[1]
        ctrlData[4] = hindLeg_left_q[0]
        ctrlData[5] = hindLeg_left_q[1]
        ctrlData[6] = hindLeg_right_q[0]
        ctrlData[7] = hindLeg_right_q[1]
        theMouse.runStep(ctrlData)

    for i in range(NUM):
        foreLeg_left_q = theController.get_leg_ctrl(0, leg_point_x[0], leg_point_y[0] + 0.025 / NUM * i)
        hindLeg_left_q = theController.get_leg_ctrl(2, leg_point_x[2] - 0.05, leg_point_y[2] + 0.025 / NUM * i)
        ctrlData[0] = foreLeg_left_q[0]
        ctrlData[1] = foreLeg_left_q[1]
        ctrlData[4] = hindLeg_left_q[0]
        ctrlData[5] = hindLeg_left_q[1]
        theMouse.runStep(ctrlData)

    for i in range(NUM):
        hindLeg_right_q = theController.get_leg_ctrl(3, leg_point_x[3] - 0.05 / NUM * i,
                                                     leg_point_y[3] + 0.05 / NUM * i)
        ctrlData[6] = hindLeg_right_q[0]
        ctrlData[7] = hindLeg_right_q[1]
        theMouse.runStep(ctrlData)

    for i in range(NUM):
        foreLeg_left_q = theController.get_leg_ctrl(0, leg_point_x[0], leg_point_y[0] + 0.025 - 0.025 / NUM * i)
        hindLeg_left_q = theController.get_leg_ctrl(2, leg_point_x[2] - 0.05, leg_point_y[2] + 0.025 - 0.025 / NUM * i)
        hindLeg_right_q = theController.get_leg_ctrl(3, leg_point_x[3] - 0.05, leg_point_y[3] + 0.05 - 0.05 / NUM * i)
        ctrlData[0] = foreLeg_left_q[0]
        ctrlData[1] = foreLeg_left_q[1]
        ctrlData[4] = hindLeg_left_q[0]
        ctrlData[5] = hindLeg_left_q[1]
        ctrlData[6] = hindLeg_right_q[0]
        ctrlData[7] = hindLeg_right_q[1]
        theMouse.runStep(ctrlData)


def control_2(theMouse,ctrlData):
    print("control2")
    ctrlData[4:]=[0.0, -1.2, 0.0, -1.2, 0, 0, 0, 0]
    for i in range(10):
        #ctrlData = [0.0, 1.5, 0.0, 1.5, 0.0, -1.2, 0.0, -1.2, 0, 0, 0, 0]
        leg_point_x, leg_point_y = theMouse.runStep(ctrlData)
    NUM = 100

    for i in range(NUM):
        foreLeg_right_q = theController.get_leg_ctrl(1, leg_point_x[1], leg_point_y[1] + 0.025 / NUM * i)
        hindLeg_right_q = theController.get_leg_ctrl(3, leg_point_x[3], leg_point_y[3] + 0.025 / NUM * i)
        ctrlData[2] = foreLeg_right_q[0]
        ctrlData[3] = foreLeg_right_q[1]
        ctrlData[6] = hindLeg_right_q[0]
        ctrlData[7] = hindLeg_right_q[1]
        theMouse.runStep(ctrlData)

    for i in range(NUM):
        hindLeg_left_q = theController.get_leg_ctrl(2, leg_point_x[2] + 0.025 / NUM * i,
                                                    leg_point_y[2] + 0.03 / NUM * i)
        ctrlData[4] = hindLeg_left_q[0]
        ctrlData[5] = hindLeg_left_q[1]
        theMouse.runStep(ctrlData)

    for i in range(NUM):
        hindLeg_left_q = theController.get_leg_ctrl(2, leg_point_x[2] + 0.025 - 0.07 / NUM * i, leg_point_y[2] + 0.03)
        ctrlData[4] = hindLeg_left_q[0]
        ctrlData[5] = hindLeg_left_q[1]
        theMouse.runStep(ctrlData)

    for i in range(NUM):
        foreLeg_right_q = theController.get_leg_ctrl(1, leg_point_x[1], leg_point_y[1] + 0.025 - 0.025 / NUM * i)
        hindLeg_right_q = theController.get_leg_ctrl(3, leg_point_x[3], leg_point_y[3] + 0.025 - 0.025 / NUM * i)
        hindLeg_left_q = theController.get_leg_ctrl(2, leg_point_x[2] - 0.035, leg_point_y[2] + 0.03 - 0.03 / NUM * i)
        ctrlData[2] = foreLeg_right_q[0]
        ctrlData[3] = foreLeg_right_q[1]
        ctrlData[4] = hindLeg_left_q[0]
        ctrlData[5] = hindLeg_left_q[1]
        ctrlData[6] = hindLeg_right_q[0]
        ctrlData[7] = hindLeg_right_q[1]
        theMouse.runStep(ctrlData)

    for i in range(NUM):
        foreLeg_left_q = theController.get_leg_ctrl(0, leg_point_x[0], leg_point_y[0] + 0.025 / NUM * i)
        hindLeg_left_q = theController.get_leg_ctrl(2, leg_point_x[2] - 0.035, leg_point_y[2] + 0.025 / NUM * i)
        ctrlData[0] = foreLeg_left_q[0]
        ctrlData[1] = foreLeg_left_q[1]
        ctrlData[4] = hindLeg_left_q[0]
        ctrlData[5] = hindLeg_left_q[1]
        theMouse.runStep(ctrlData)

    for i in range(NUM):
        hindLeg_right_q = theController.get_leg_ctrl(3, leg_point_x[3] + 0.025 / NUM * i,
                                                     leg_point_y[3] + 0.03 / NUM * i)
        ctrlData[6] = hindLeg_right_q[0]
        ctrlData[7] = hindLeg_right_q[1]
        theMouse.runStep(ctrlData)

    for i in range(NUM):
        hindLeg_right_q = theController.get_leg_ctrl(3, leg_point_x[3] + 0.025 - 0.07 / NUM * i,
                                                     leg_point_y[3] + 0.03)
        ctrlData[6] = hindLeg_right_q[0]
        ctrlData[7] = hindLeg_right_q[1]
        theMouse.runStep(ctrlData)

    for i in range(NUM):
        foreLeg_left_q = theController.get_leg_ctrl(0, leg_point_x[0], leg_point_y[0] + 0.025 - 0.025 / NUM * i)
        hindLeg_left_q = theController.get_leg_ctrl(2, leg_point_x[2] - 0.035, leg_point_y[2] + 0.025 - 0.025 / NUM * i)
        hindLeg_right_q = theController.get_leg_ctrl(3, leg_point_x[3] - 0.035, leg_point_y[3] + 0.03 - 0.03 / NUM * i)
        ctrlData[0] = foreLeg_left_q[0]
        ctrlData[1] = foreLeg_left_q[1]
        ctrlData[4] = hindLeg_left_q[0]
        ctrlData[5] = hindLeg_left_q[1]
        ctrlData[6] = hindLeg_right_q[0]
        ctrlData[7] = hindLeg_right_q[1]
        theMouse.runStep(ctrlData)


if __name__ == '__main__':
    parser = argparse.ArgumentParser("Description.")
    parser.add_argument('--fre', default=0.67,
                        type=float, help="Gait stride")  # 设置步幅
    args = parser.parse_args()
    # theMouse = SimModel("./models/my_test1.xml")
    theMouse = SimModel("./models/scene_test1.xml")
    theController = Controller(args.fre)
    start = time.time()
    # ctrlData=[0]*12
    ctrlData = [0.0, 1.5, 0.0, 1.5, 0.0, -1.2, 0.0, -1.2, 0, 0, 0, 0]
    for i in range(500):
        leg_point_x, leg_point_y = theMouse.runStep(ctrlData)
    print(leg_point_x)
    print(leg_point_y)

    dis_list = []
    size = 500
    threshold = 0.02

    theMouse.initializing()
    start = time.time()
    for i in range(RUN_STEPS):
        ctrlData = theController.runStep()
        theMouse.runStep(ctrlData)
        sensor1, _ = theMouse.get_sensors()
        dis = -sensor1['pos'][1]
        dis_list.append(dis)
        if len(dis_list) > size:
            dis_list.pop(0)
            dis_dis = dis_list[-1] - dis_list[0]
            if dis_dis < threshold:
                control_2(theMouse,ctrlData)
                ctrlData = [0.0, 1.5, 0.0, 1.5, 0.0, -1.2, 0.0, -1.2, 0, 0, 0, 0]
                theMouse.runStep(ctrlData)
                theController.initial()
                dis_list = []
                for i in range(500):
                    ctrlData = theController.runStep()

    end = time.time()
    timeCost = end - start
    print("Time -> ", timeCost)
# glfw.terminate()

# theMouse.write_log()

