import math
from LegModel.foreLeg import ForeLegM
from LegModel.hindLeg import HindLegM


def get_Bezier_point(tao, psi, delta, St): # tao=前进步长的一半,psi=地上的最高点,delta=地下的最低点
    St = St % 2

    reverse = False
    if reverse:
        St=2-St

    control_point_list=[
        [-tao, 0],
        [-1.4 * tao, 0],
        [-1.5 * tao, 0.9 * psi],
        [-1.5 * tao, 0.9 * psi],
        [-1.5 * tao, 0.9 * psi],
        [0, 0.9 * psi],
        [0, 0.9 * psi],
        [0, 1.1 * psi],
        [1.5 * tao, 1.1 * psi],
        [1.5 * tao, 1.1 * psi],
        [1.4 * tao, 0],
        [tao, 0]
    ]
    n = len(control_point_list)
    if St>=0 and St<1:
        return [tao * (1 - 2 * St),delta * math.cos(math.pi * (1 - 2 * St)/2)]
    elif St>=1 and St<=2:
        x, y = 0, 0
        for j in range(n):
                x += math.factorial(n - 1) / math.factorial(j) / math.factorial(n - 1 - j) * math.pow(
                    (St - 1), j) * math.pow(1 - (St - 1), n - 1 - j) * control_point_list[j][0]
                y += math.factorial(n - 1) / math.factorial(j) / math.factorial(n - 1 - j) * math.pow(
                    (St - 1), j) * math.pow(1 - (St - 1), n - 1 - j) * control_point_list[j][1]
        return x,y


fl_params = {'lr0': 0.033, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.0295,
             'l2': 0.0145, 'l3': 0.0225, 'l4': 0.0145, 'alpha': 23 * math.pi / 180}
fl = ForeLegM(fl_params)
hl_params = {'lr0': 0.032, 'rp': 0.008, 'd1': 0.0128, 'l1': 0.0317,
             'l2': 0.02, 'l3': 0.0305, 'l4': 0.0205, 'alpha': 73 * math.pi / 180}
hl = HindLegM(hl_params)

def pos_2_angle(x,y,leg="f"):
    if leg=="f":
        return fl.pos_2_angle(x,y)
    elif leg=="h":
        return hl.pos_2_angle(x,y)