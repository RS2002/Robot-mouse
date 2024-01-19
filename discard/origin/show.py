from ToSim import SimModel
import numpy as np

theMouse = SimModel("./models/scene_test1.xml")
path="./result/scene1/PPO_without_pretrain/ctrldata.npz"
theMouse.initializing()

ctrldata=np.load(path)
ctrldata=ctrldata['ctrldata']
for i in range(len(ctrldata)//12):
    ctrlData=ctrldata[i*12:(i+1)*12]
    theMouse.runStep(ctrlData)
