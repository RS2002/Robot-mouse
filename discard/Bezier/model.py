import math
import matplotlib.pyplot as plt
import numpy as np

def Bezier_2D(control_point_list, plot=True, point_num=int(1e3)):
    n = len(control_point_list) # 控制点个数
    point_result = np.zeros([point_num,2])
    for i in range(point_num):
        for j in range(n):
            point_result[i,0]+=math.factorial(n-1)/math.factorial(j)/math.factorial(n-1-j)*math.pow(i/point_num,j)*math.pow(1-i/point_num,n-1-j)*control_point_list[j][0]
            point_result[i,1]+=math.factorial(n-1)/math.factorial(j)/math.factorial(n-1-j)*math.pow(i/point_num,j)*math.pow(1-i/point_num,n-1-j)*control_point_list[j][1]
    if plot:
        plt.plot(point_result[:,0],point_result[:,1],'r')
        plt.show()
    return point_result

def Bezier_Rahme(tao, psi, delta, plot=True, point_num=int(1e3)):
    point_num = point_num // 2 * 2
    control_point_list=np.array([
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

    # point_result = np.zeros([point_num, 2])
    # point_result[point_num // 2 : , : ] = Bezier_2D(control_point_list , plot=False, point_num = point_num // 2)
    # for i in range(point_num//2):
    #     point_result[i, 0] = tao * (1 - 2 * i / (point_num // 2))
    #     point_result[i, 1] = delta * math.cos(math.pi * (1 - 2 * i / (point_num // 2))/2)

    point_result = np.zeros([point_num, 2])
    for i in range(point_num):
        point_result[i,:]=get_Bezier_point(tao, delta, control_point_list, 2*i/point_num)

    if plot:
        plt.plot(point_result[:,0],point_result[:,1],'b',linewidth=4)
        plt.plot(control_point_list[:,0],control_point_list[:,1],'ro',linewidth=22)
        plt.xlabel("X", fontsize=16)
        plt.ylabel("Y", fontsize=16)
        plt.tick_params(axis='x', labelsize=12)
        plt.tick_params(axis='y', labelsize=12)
        plt.show()
    return point_result



def get_Bezier_point(tao, delta, control_point_list, St):
    St = St % 2
    n = len(control_point_list)
    if St>=0 and St<1:
        return [tao * (1 - 2 * St),delta * math.cos(math.pi * (1 - 2 * St)/2)]
    elif St>=1 and St<2:
        x, y = 0, 0
        for j in range(n):
                x += math.factorial(n - 1) / math.factorial(j) / math.factorial(n - 1 - j) * math.pow(
                    (St - 1), j) * math.pow(1 - (St - 1), n - 1 - j) * control_point_list[j][0]
                y += math.factorial(n - 1) / math.factorial(j) / math.factorial(n - 1 - j) * math.pow(
                    (St - 1), j) * math.pow(1 - (St - 1), n - 1 - j) * control_point_list[j][1]
        return x,y


if __name__ == '__main__':
    '''control_point_list=[[1,2],[2,3],[3,1]]
    Bezier_2D(control_point_list)'''
    Bezier_Rahme(1,0.5,-0.05)