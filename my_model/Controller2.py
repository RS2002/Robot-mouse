import numpy as np
import math

from LegModel.forPath import LegPath
from LegModel.foreLeg import ForeLegM
from LegModel.hindLeg import HindLegM
from Controller import MouseController

class Controller(MouseController):
    def __init__(self,freq):
        super().__init__(freq)

    def get_leg_ctrl(self,leg_id,x,y): #获取舵机转动角度的函数，leg_id为腿部编号(0-3表示左前、右前、左后、右后)
        if leg_id==0:
            leg=self.fl_left
        elif leg_id==1:
            leg=self.fl_right
        elif leg_id==2:
            leg=self.hl_left
        elif leg_id==3:
            leg=self.hl_right
        else:
            return [0,0]
        return leg.pos_2_angle(x,y)

