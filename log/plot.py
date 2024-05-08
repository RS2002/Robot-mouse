import matplotlib.pyplot as plt

index="position"
linewidth=2.5

def parse(path,content):
    x,y,z=[],[],[]
    path=path+"/"+content+".txt"
    with open(path,'r') as file:
        for line in file:
            line=line[1:]
            line=line.split()
            x.append(abs(eval(line[0])))
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
plt.ylabel("Distance", fontsize=16)
plt.xlabel("Step", fontsize=16)
plt.legend(prop={'size': 18}, framealpha=0.3)
plt.title("X Direction Displacement", fontsize=16)
plt.show()