import matplotlib.pyplot as plt

'''data=[]
target="scene0"
f=open("./data/PPO/"+target+".txt")
for line in f:
    index=line.index(":")
    data.append(eval(line[index+1:]))
plt.plot(data)
plt.title("Flat Ground")
plt.ylabel("distance")
plt.xlabel("episode")
plt.show()
'''

data=[]
target="scene1"
f=open("./data/PPO/"+target+".txt")
for line in f:
    index=line.index(":")
    data.append(eval(line[index+1:]))
plt.plot(data)
plt.title("Obstacle")
plt.ylabel("distance")
plt.xlabel("episode")
plt.show()
