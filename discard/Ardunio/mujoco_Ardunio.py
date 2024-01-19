import mujoco
import mujoco.viewer
from LegModel.foreLeg import ForeLegM
import numpy as np
import matplotlib.pyplot as plt
import math
from pinpong.board import Board,Pin,Servo


Board("uno").begin()
s1=Servo(Pin(Pin.D9))
s2=Servo(Pin(Pin.D10))

model = mujoco.MjModel.from_xml_path("../models/FL.xml")
data = mujoco.MjData(model)
viewer = mujoco.viewer.launch_passive(model, data)
fl_params = {'lr0': 0.033, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.0295,
             'l2': 0.0145, 'l3': 0.0225, 'l4': 0.0145, 'alpha': 23 * np.pi / 180}
fl_left = ForeLegM(fl_params)

index=3
point=np.array([[0,0.01],[0,-0.005],[0.03,0],[-0.03,0]])
dx, dy = 0, -0.045
x=point[index,0]+dx
y=point[index,1]+dy
ctrl=fl_left.pos_2_angle(x , y)
real_ctrl=[(90 + (int(ctrl[0] * 180 / math.pi))) % 181,(90-(int(ctrl[1] * 180 / math.pi))) % 181]
print(real_ctrl)

while True:
    data.ctrl=ctrl
    mujoco.mj_step(model, data)
    viewer.sync()

    s1.write_angle(real_ctrl[0])
    s2.write_angle(real_ctrl[1])
