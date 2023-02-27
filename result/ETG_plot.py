import matplotlib.pyplot as plt
import scipy.io as scio
import numpy as np

'''def plot_vel_y():
    target = "linear velocity"
    f1 = open("./data/flat_ground/" + target + ".txt")
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
    plt.plot(data1[1])
    plt.title("Flat Ground")
    plt.ylabel("Velocity in axis Y")
    plt.xlabel("Steps")
    plt.show()'''

'''def plot_reward_RL():
    f = open("./data/ETG_RL/flat ground/log.log", 'r')
    step = []
    reward = []
    for lines in f:
        if "Total Steps" in lines:
            content = lines.split(" ")
            # print(content)
            temp_step = eval(content[-3])
            temp_reward=eval(content[-1][:-1])
            step.append(temp_step)
            reward.append(temp_reward)
    plt.plot(step, reward)
    plt.xlabel("Trainnig steps")
    plt.ylabel("Reward")
    plt.title("Reward of flat ground")
    plt.show()

def plot_reward_ETG():
    f = open("./data/ETG_RL/flat ground/log.log",'r')
    step=[]
    temp_step=0
    reward=[]
    for lines in f:
        if "Total Steps" in lines:
            content=lines.split(" ")
            #print(content)
            temp_step=eval(content[-3])
        if "ESSteps" in lines:
            content = lines.split(" ")
            #print(content)
            temp_reward = eval(content[-5])
            #print(temp_reward)
            if len(step)==0 or temp_step!=step[-1]:
                reward.append(temp_reward)
                step.append(temp_step)
    plt.plot(step,reward)
    plt.xlabel("Trainnig steps")
    plt.ylabel("Reward")
    plt.title("Reward of flat ground")
    plt.show()'''



def plot_reward():
    f = open("./s1_test_pre_s1/log.log",'r')
    step=[]
    temp_step=0
    reward=[]
    for lines in f:
        if "Total Steps" in lines:
            content=lines.split(" ")
            #print(content)
            temp_step=eval(content[-3])
            '''if temp_step>1e6:
                break'''
        if "Evaluation" in lines:
            content = lines.split(" ")
            #print(content)
            temp_reward = eval(content[-4])
            #print(temp_reward)
            reward.append(temp_reward)
            step.append(temp_step)
    plt.plot(step,reward)
    plt.xlabel("Steps")
    plt.ylabel("Reward")
    plt.title("ETG-RL")
    plt.show()
    step=np.array(step)
    reward=np.array(reward)
    scio.savemat("ETG_RL.mat",{'step':step,'reward':reward})


if __name__=='__main__':
    #plot_vel_y()
    plot_reward()
    #plot_reward_RL()
    #plot_reward_ETG()