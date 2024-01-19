import time
from LegModel.hindLeg import HindLegM
import numpy as np
import math
import socket

s_address = ('10.1.1.103', 6666)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
theString="0,0,0,0,0,0,0,0,0"



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

def getOvalPathPoint( radian, leg_flag="H", halfPeriod=1):
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

tao = 0.015 #前进步长的一半
psi = 0.005 #地上的最高点
delta = -0.0025 #地下的最低点
dx, dy = 0, -0.05 #初始坐标
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


hl_params = {'lr0': 0.032, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.0317,
             'l2': 0.02, 'l3': 0.0305, 'l4': 0.0205, 'alpha': 73 * np.pi / 180}
hl_right = HindLegM(hl_params)



right=False


num=65
while(True):
    for t in range(num*2):
        St=t/num
        # x,y=get_Bezier_point(tao, delta, control_point_list, St, dx, dy)
        # x,y=get_circle_point(r=0.01,t=St,dx=dx,dy=dy)
        x, y = getOvalPathPoint(St)
        ctrl = hl_right.pos_2_angle(x , y)
        if right:
            ctrl[0]=-ctrl[0]*180/math.pi
            ctrl[1]=ctrl[1]*180/math.pi
        else:
            ctrl[0]=ctrl[0]*180/math.pi
            ctrl[1]=-ctrl[1]*180/math.pi
        print(ctrl)
        ctrl_string=str(ctrl[0])+","+str(ctrl[1])+","+theString
        sock.sendto(ctrl_string.encode(), s_address)
        time.sleep(0.05)