import matplotlib.pyplot as plt
import numpy as np
index="acceleration"
linewidth=2.5

def parse(path,content):
    x,y,z=[],[],[]
    path=path+"/"+content+".txt"
    with open(path,'r') as file:
        for line in file:
            line=line[1:]
            line=line.split()
            x.append((eval(line[0])))
            y.append(eval(line[1]))
            if line[2][-1]==":":
                line[2]=line[2][:-2]
            z.append(eval(line[2]))
    return x,y,z

x_c,y_c,z_c=parse("scene0/complex",index)
x_s,y_s,z_s=parse("scene0/simple",index)
x_cr,y_cr,z_cr=parse("scene0/random-complex",index)
x_sr,y_sr,z_sr=parse("scene0/random-simple",index)

e=list(range(len(x_c)))

plt.plot(e,x_c,label="complex", linewidth=linewidth)
plt.plot(e,x_s,label="simple", linewidth=linewidth)
plt.plot(e,x_cr,label="complex, random", linewidth=linewidth)
plt.plot(e,x_sr,label="simple, random", linewidth=linewidth)
plt.ylabel("Acceleration", fontsize=16)
plt.xlabel("Step", fontsize=16)
plt.legend(prop={'size': 18}, framealpha=0.3)
plt.title("X", fontsize=16)
plt.show()

print(np.std(x_c),np.std(x_s),np.std(x_cr),np.std(x_sr))

plt.plot(e,y_c,label="complex", linewidth=linewidth)
plt.plot(e,y_s,label="simple", linewidth=linewidth)
plt.plot(e,y_cr,label="complex, random", linewidth=linewidth)
plt.plot(e,y_sr,label="simple, random", linewidth=linewidth)
plt.ylabel("Acceleration", fontsize=16)
plt.xlabel("Step", fontsize=16)
plt.legend(prop={'size': 18}, framealpha=0.3)
plt.title("Y", fontsize=16)
plt.show()

print(np.std(y_c),np.std(y_s),np.std(y_cr),np.std(y_sr))

plt.plot(e,z_c,label="complex", linewidth=linewidth)
plt.plot(e,z_s,label="simple", linewidth=linewidth)
plt.plot(e,z_cr,label="complex, random", linewidth=linewidth)
plt.plot(e,z_sr,label="simple, random", linewidth=linewidth)
plt.ylabel("Acceleration", fontsize=16)
plt.xlabel("Step", fontsize=16)
plt.legend(prop={'size': 18}, framealpha=0.3)
plt.title("Z", fontsize=16)
plt.show()

print(np.std(z_c),np.std(z_s),np.std(z_cr),np.std(z_sr))
