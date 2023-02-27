import numpy as np
import matplotlib.pyplot as plt

path=np.load("./s0_test_pre/path_save.npz")
path=path['path']
x=[]
y=[]
for i in range(5001):
    x.append(path[i*2])
    y.append(path[i*2+1])
plt.plot(x,y)
plt.show()
