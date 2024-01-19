import matplotlib.pyplot as plt
import numpy as np
import re

# def parse(path):
#     pos_list=[]
#     vel_list=[]
#     with open(path,'r') as file:
#         for line in file:
#             result = re.findall(r"\]:\s.*?\s", line)
#             pos=result[0][2:]
#             vel=result[1][2:]
#             pos_list.append(eval(pos))
#             vel_list.append(eval(vel))
#     return pos_list,vel_list
#
#
# pos_b,vel_b=parse("./log/log.txt")
# pos_o,vel_o=parse("../origion (mujoco version)/log/log.txt")


def parse(path):
    pos_list=[]
    vel_list=[]
    pos_path=path+"position.txt"
    with open(pos_path,'r') as file:
        for line in file:
            line=line.split()
            if line[0]=="[":
                pos=line[2]
            else:
                pos=line[1]
            pos_list.append(eval(pos))
    vel_path=path+"linear velocity.txt"
    with open(vel_path,'r') as file:
        for line in file:
            line=line.split()
            if line[0]=="[":
                vel=line[2]
            else:
                vel=line[1]
            vel_list.append(eval(vel))
    return pos_list,vel_list
pos_b,vel_b=parse("./log/")
pos_o,vel_o=parse("../origion (mujoco version)/log/")

plt.plot(pos_b,'r', label="Bezier", linewidth=2.5)
plt.plot(pos_o,'b', label="Oval", linewidth=2.5)
plt.xlabel('Step', fontsize=14)
plt.ylabel('Distance', fontsize=14)
plt.legend(prop={'size': 18}, framealpha=0.3)
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.show()


plt.plot(vel_b,'r', label="Bezier", linewidth=2.5)
plt.plot(vel_o,'b', label="Oval", linewidth=2.5)
plt.xlabel('Step', fontsize=14)
plt.ylabel('Velocity', fontsize=14)
plt.legend(prop={'size': 18}, framealpha=0.3)
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.show()