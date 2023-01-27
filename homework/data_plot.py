import matplotlib.pyplot as plt
import numpy as np

def plot3dimesion():
    target="linear velocity"
    #target="angular velocity"
    #target="position"
    #target="acceleration"

    f1=open("./data/scene0/"+target+".txt")
    data1=[[],[],[],[]]
    for line in f1:
       str=line[1:]
       idx=str.index(':')
       str1=str[:idx-1].split()
       str2=str[idx+1:].split()
       for i in range(3):
           data1[i].append(eval(str1[i]))
       data1[-1].append(eval(str2[0]))
    f1.close()

    #f2=open("./data/scene_test1/"+target+".txt")
    #f2=open("./data/scene_test2/"+target+".txt")
    f2=open("./data/scene_test2pro/"+target+".txt")
    #f2=open("./data/scene_test3/"+target+".txt")
    data2=[[],[],[],[]]
    for line in f2:
       str=line[1:]
       idx=str.index(':')
       str1=str[:idx-1].split()
       str2=str[idx+1:].split()
       for i in range(3):
           data2[i].append(eval(str1[i]))
       data2[-1].append(eval(str2[0]))
    f2.close()

    for i in range(4):
        #plt.plot(data1[i],'.r',label="base")
        #plt.plot(data2[i],'.b',label="test1")
        plt.plot(data1[i],'r',label="base")
        plt.plot(data2[i],'b',label="test1")
        if i<3:
            temp=chr(ord('x')+i)
        else:
            temp="value"
        plt.title(target+"_"+temp)
        plt.legend()
        plt.show()

def plot1dimension():
    target="touch_fl"
    #target="touch_fr"
    #target="touch_rl"
    #target="touch_rr"
    f1 = open("./data/scene0/" + target + ".txt")
    data1 = []
    for line in f1:
        data1.append(eval(line))
    f1.close()

    #f2=open("./data/scene_test1/"+target+".txt")
    #f2=open("./data/scene_test2/" + target + ".txt")
    #f2=open("./data/scene_test2pro/"+target+".txt")
    f2=open("./data/scene_test3/"+target+".txt")
    data2 = []
    for line in f2:
        data2.append(eval(line))
    f2.close()

    # plt.plot(data1,'.r',label="base")
    # plt.plot(data2,'.b',label="test1")
    plt.plot(data1, 'r', label="base")
    plt.plot(data2, 'b', label="test1")
    plt.title(target)
    plt.legend()
    plt.show()

def compute_fft():
    #target = "linear velocity"
    target="angular velocity"
    #target="position"
    #target="acceleration"

    f1 = open("./data/scene0/" + target + ".txt")
    data1 = [[], [], [], []]
    for line in f1:
        str = line[1:]
        idx = str.index(':')
        str1 = str[:idx - 1].split()
        str2 = str[idx + 1:].split()
        for i in range(3):
            data1[i].append(eval(str1[i]))
        data1[-1].append(eval(str2[0]))
    f1.close()

    f2=open("./data/scene_test1/"+target+".txt")
    #f2=open("./data/scene_test2/"+target+".txt")
    #f2 = open("./data/scene_test2pro/" + target + ".txt")
    #f2=open("./data/scene_test3/"+target+".txt")
    data2 = [[], [], [], []]
    for line in f2:
        str = line[1:]
        idx = str.index(':')
        str1 = str[:idx - 1].split()
        str2 = str[idx + 1:].split()
        for i in range(3):
            data2[i].append(eval(str1[i]))
        data2[-1].append(eval(str2[0]))
    f2.close()

    dimension = 2
    x1 = data1[dimension][2000:]
    x2 = data2[dimension][2000:]
    y1 = np.abs(np.fft.fft(x1)) / len(x1)
    y1[y1 < 0.01 * max(y1)] = 0
    y2 = np.abs(np.fft.fft(x2)) / len(x2)
    y2[y2 < 0.01 * max(y2)] = 0
    #plt.plot(y1, 'r', label="base")
    plt.plot(y2, 'b', label="test1")
    plt.plot(y1, 'r', label="base")
    plt.title("FFT")
    plt.legend()
    plt.show()




if __name__=='__main__':
    #plot3dimesion()
    #plot1dimension()
    compute_fft()