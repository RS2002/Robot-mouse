from ToSim import SimModel
import numpy as np

'''theMouse = SimModel("./models/dynamic_4l_t3.xml")
path="./ctrldata/ground.npz"'''
theMouse = SimModel("./models/ETG_obstacle1.xml")
path="./ctrldata/obstacle1.npz"
theMouse.initializing()

ctrldata=np.load(path)
ctrldata=ctrldata['ctrldata']
for i in range(len(ctrldata)//12):
    ctrlData=ctrldata[i*12:(i+1)*12]
    theMouse.runStep(ctrlData)
theMouse.write_log()
