import mujoco
import mujoco.viewer
from LegModel.foreLeg import ForeLegM
import numpy as np
import matplotlib.pyplot as plt
import math

def get_Bezier_point(tao, delta, control_point_list, St, dx=0.0, dy=0.0):
    '''control_point_list[:, 0] += dx
    control_point_list[:, 1] += dy'''
    St = St % 2
    n = len(control_point_list)
    if St>=0 and St<1:
        return tao * (1 - 2 * St)+dx,delta * math.cos(math.pi * (1 - 2 * St)/2)+dy
    elif St>=1 and St<2:
        x, y = 0, 0
        for j in range(n):
                x += math.factorial(n - 1) / math.factorial(j) / math.factorial(n - 1 - j) * math.pow(
                    (St - 1), j) * math.pow(1 - (St - 1), n - 1 - j) * control_point_list[j][0]
                y += math.factorial(n - 1) / math.factorial(j) / math.factorial(n - 1 - j) * math.pow(
                    (St - 1), j) * math.pow(1 - (St - 1), n - 1 - j) * control_point_list[j][1]
        return x+dx,y+dy

tao = 0.03 #前进步长的一半
psi = 0.01 #地上的最高点
delta = -0.005 #地下的最低点
dx, dy = 0, -0.045 #初始坐标
control_point_list = np.array([ #控制点列表
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

x_target=[]
y_target=[]
x_real=[]
y_real=[]


model = mujoco.MjModel.from_xml_path("../models/FL.xml")
data = mujoco.MjData(model)
viewer = mujoco.viewer.launch_passive(model, data)
fl_params = {'lr0': 0.033, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.0295,
             'l2': 0.0145, 'l3': 0.0225, 'l4': 0.0145, 'alpha': 23 * np.pi / 180}
fl_left = ForeLegM(fl_params)

#dq1,dq2=0,0
#DX,DY=0,0
for i in range(2):
    for t in range(1000):
        St=t/500
        x,y=get_Bezier_point(tao, delta, control_point_list, St, dx, dy)

        '''[q1,q2]=fl_left.pos_2_angle(x, y)
        if i==1:
            q1+=dq1/20
            q2+=dq2/20
        data.ctrl = [q1,q2]'''
        '''for j in range(10):
            data.ctrl = fl_left.pos_2_angle(x, y)'''

        #data.ctrl = fl_left.pos_2_angle(x+DX, y+DY)

        data.ctrl = fl_left.pos_2_angle(x , y)
        mujoco.mj_step(model,data)
        viewer.sync()

        originPoint = data.site("router_shoulder_fl").xpos
        currentPoint = data.site("foot_s_fl").xpos
        #print(originPoint, currentPoint)
        tX = currentPoint[1] - originPoint[1]
        tY = currentPoint[2] - originPoint[2]

        '''real_q1=data.sensor("m1_fl").data[0]
        real_q2=data.sensor("m2_fl").data[0]
        dq1=q1-real_q1
        dq2=q2-real_q2'''

        # if i==1:
        #     DX+=x-tX
        #     DY+=y-tY


        x_target.append(x)
        y_target.append(y)
        x_real.append(tX)
        y_real.append(tY)


plt.plot(x_target,y_target,'r')
plt.plot(x_real,y_real,'b')
plt.show()
