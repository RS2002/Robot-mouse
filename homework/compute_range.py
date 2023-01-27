import math
import matplotlib.pyplot as plt


def LawOfCosines_edge(la, lb, angle_ab):
    lc_2 = la * la + lb * lb - 2 * la * lb * math.cos(angle_ab)
    lc = math.sqrt(lc_2)
    return lc


def LawOfCosines_angle(la, lb, lc):
    angle_ab_cos = (la * la + lb * lb - lc * lc) / (2 * la * lb)
    #print(angle_ab_cos)
    angle_ab = math.acos(angle_ab_cos)
    return angle_ab

def compute_frontleg_range(params):
    lr0 = params['lr0']
    rp = params['rp']
    d1 = params['d1']
    l1 = params['l1']
    l2 = params['l2']
    l3 = params['l3']
    l4 = params['l4']
    alpha = params['alpha']
    l_a = LawOfCosines_edge(l3, l4, (math.pi - alpha))
    beta = LawOfCosines_angle(l_a, l3, l4)

    #lb_max=lr0-(d1**2-rp**2)**(1/2)
    lb_max = lr0 - (d1 - rp)
    #lb_min = lb_max - 2 * math.pi * rp
    #theta2_t_min = LawOfCosines_angle(l1, l2, lb_min) - beta
    theta2_t_min = 10**(-5)
    theta2_t_max = LawOfCosines_angle(l1, l2, lb_max) - beta
    '''print(theta2_t_max)
    print(theta2_t_min)'''

    #print(compute_D(l1,l_a,theta2_t_max,math.pi/2))

    x=[]
    y=[]
    for i in range(500):
        for j in range(500):
            theta2_t=theta2_t_min+i/499*(theta2_t_max-theta2_t_min)
            #q1=2*j/499*math.pi-math.pi
            q1=j/499*math.pi
            D=compute_D(l1,l_a,theta2_t,q1)
            x.append(D[0])
            y.append(D[1])
            #y.append(D[1]-d1)
    plt.plot(x,y,'.')
    plt.show()

def compute_D(l1,la,theta2_t,q1):
    lt=LawOfCosines_edge(la, l1, theta2_t)
    theta1_t1=LawOfCosines_angle(l1, lt, la)
    #print(theta1_t1)
    theta1_t2=q1-theta1_t1
    x=lt*math.sin(theta1_t2)
    y=-lt*math.cos(theta1_t2)
    return [x,y]


if __name__=='__main__':
    params = {'lr0': 0.033, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.0295,'l2': 0.0145, 'l3': 0.0225, 'l4': 0.0145, 'alpha': 23 * math.pi / 180}
    compute_frontleg_range(params)


