import mujoco
import mujoco.viewer
from LegModel.hindLeg import HindLegM
import numpy as np


# model = mujoco.MjModel.from_xml_path("../models/FL.xml")
# data = mujoco.MjData(model)
# fl_params = {'lr0': 0.033, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.0295,
#              'l2': 0.0145, 'l3': 0.0225, 'l4': 0.0145, 'alpha': 23 * np.pi / 180}
# fl_left = ForeLegM(fl_params)
#
# data.ctrl = [0,0]
# mujoco.mj_step(model, data)
# viewer = mujoco.viewer.launch(model, data)


model = mujoco.MjModel.from_xml_path("../models/RR.xml")
data = mujoco.MjData(model)
viewer = mujoco.viewer.launch_passive(model, data)
hl_params = {'lr0': 0.032, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.0317,
             'l2': 0.02, 'l3': 0.0305, 'l4': 0.0205, 'alpha': 73 * np.pi / 180}
hl_right = HindLegM(hl_params)


while True:
    data.ctrl = [0, 0]
    mujoco.mj_step(model, data)
    viewer.sync()
