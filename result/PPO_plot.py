import matplotlib.pyplot as plt
import scipy.io as scio
import numpy as np
step=[]
reward=[]
with open("./PPO/PPO_s4.txt") as file:
    for line in file:
        str=line.split()
        step_temp=eval(str[1])
        reward_temp=eval(str[-1])
        if step_temp>8e6:
            break
        step.append(step_temp)
        reward.append(reward_temp)
plt.plot(step,reward)
plt.xlabel("Step")
plt.ylabel("Reward")
plt.title("PPO")
plt.show()
step=np.array(step)
reward=np.array(reward)
scio.savemat("PPO.mat",{'step':step,'reward':reward})
