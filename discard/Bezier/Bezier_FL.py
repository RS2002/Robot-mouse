import mujoco
import mujoco.viewer
from LegModel.foreLeg import ForeLegM
import numpy as np
import matplotlib.pyplot as plt
import math

def get_range():
    fl_params = {'lr0': 0.033, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.0295,
                 'l2': 0.0145, 'l3': 0.0225, 'l4': 0.0145, 'alpha': 23 * np.pi / 180}
    fl_left = ForeLegM(fl_params)
    X=[]
    Y=[]
    num=3000
    for i in range(num):
        for j in range(num):
            q1,q2=fl_left.pos_2_angle(-1+2/num*i,-1+2/num*j)
            if (q1!=0 or q2!=0) and (q1>=-math.pi/2 and q1 <= math.pi/2) and (q2>=-math.pi/2 and q2 <= math.pi/2):
                X.append(-1+2/num*i)
                Y.append(-1+2/num*j)
    '''plt.plot(X,Y,'.')
    plt.show()'''
    return X,Y


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

def get_circle_point(r, t, dx=0.0, dy=0.0):
    t=t%2
    angle=math.pi*t
    x=r*math.cos(angle)
    y=r*math.sin(angle)
    return x+dx,y+dy



para_FU = [[-0.00, -0.045], [0.03, 0.01]]
para_FD = [[-0.00, -0.045], [0.03, 0.005]]
para_HU = [[0.00, -0.05], [0.03, 0.01]]
para_HD = [[0.00, -0.05], [0.03, 0.005]]
def getOvalPathPoint( radian, leg_flag="F", halfPeriod=1):
    radian=radian*math.pi
    if leg_flag == "F":
        if radian < halfPeriod * math.pi:
            pathParameter = para_FU
            cur_radian = radian / halfPeriod
        else:
            pathParameter = para_FD
            cur_radian = (radian) / (2 - halfPeriod)
    else:
        if radian < halfPeriod * math.pi:
            pathParameter = para_HU
            cur_radian = radian / halfPeriod
        else:
            pathParameter = para_HD
            cur_radian = (radian) / (2 - halfPeriod)
    originPoint = pathParameter[0]
    ovalRadius = pathParameter[1]
    # 根据椭圆方程和角度求坐标位置
    trg_x = originPoint[0] + ovalRadius[0] * math.cos(cur_radian)
    trg_y = originPoint[1] + ovalRadius[1] * math.sin(cur_radian)
    return [trg_x, trg_y]


#dq1,dq2=0,0
#DX,DY=0,0
for i in range(5):
# while True:
    for t in range(1000):
        St=t/500
        # x,y=get_Bezier_point(tao, delta, control_point_list, St, dx, dy)
        # x, y = get_circle_point(r=0.015, t=St, dx=dx, dy=dy)
        x,y=getOvalPathPoint(St)

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

        if i!=0:
            x_target.append(x)
            y_target.append(y)
            x_real.append(tX)
            y_real.append(tY)


X,Y=get_range()
plt.plot(X,Y,'bo',label='bound')
plt.plot(x_target,y_target,'yellow',label='target', linewidth=1)
plt.plot(x_real,y_real,'r',label='real', linewidth=1)
plt.legend(prop={'size': 18}, framealpha=0.3)
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.xlabel('X', fontsize=20)
plt.ylabel('Y', fontsize=20)
plt.show()