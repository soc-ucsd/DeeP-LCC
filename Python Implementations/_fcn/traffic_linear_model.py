import math
import numpy as np

def traffic_linear_model(ID, Ts, hdv_type, measure_type, v_star):
    '''
        CLCC model

    ID:           vehicle ID
    Ts:           sampling time
    hdv_type:     type of HDV car-following model
    measure_type: measure type
    '''

    if hdv_type == 1:
        # Driver Model: OVM
        alpha = 0.6
        beta  = 0.9
        s_st  = 5
        s_go  = 35
        v_max  = 30
        # Equilibrium spacing
        s_star = math.acos(1 - v_star / v_max * 2) / math.pi * (s_go - s_st) + s_st
        # Linear coefficients
        alpha1 = alpha * v_max / 2 * math.pi / (s_go - s_st) * math.sin(math.pi * (s_star - s_st) / (s_go - s_st))
        alpha2 = alpha + beta
        alpha3 = beta
    elif hdv_type == 2:
        # Driver Model: IDM
        v_max  = 30
        T_gap  = 1
        a      = 1
        b      = 1
        delta  = 4
        s_st   = 5
        # Equilibrium spacing
        s_star = (s_st + T_gap * v_star) / math.sqrt(1 - (v_star / v_max) ** delta)
        # Linear coefficients
        alpha1 = 2 * a * (s_st + T_gap * v_star) ** 2 / s_star ** 3
        alpha2 = math.sqrt(a / b) * v_star * (s_st + T_gap * v_star) / s_star ** 2 + 2 * a * (2 * v_star ** 3) / v_max ** 4 + T_gap * (s_st + T_gap * v_star) / s_star **2
        alpha3 = math.sqrt(a / b) * v_star * (s_st + T_gap * v_star) / s_star ** 2

    pos_cav    = np.argwhere(np.ravel(ID) == 1)   # position of CAVs
    n_vehicle  = len(ID)                          # number of vehicles
    n_cav      = len(pos_cav)                     # number of CAVs

    A = np.zeros((n_vehicle * 2, n_vehicle * 2))

    P1 = [[0, -1], [alpha1, -alpha2]]
    P2 = [[0, 1], [0, alpha3]]
    S1 = [[0, -1], [0, 0]]
    S2 = [[0, 1], [0, 0]]

    A[0:2, 0:2] = P1
    for i in range(2, n_vehicle+1):
        if ID[i-1] == 0:
            A[(2 * i - 1) - 1 : (2 * i) , (2 * i - 1) - 1 : (2 * i)]     = P1
            A[(2 * i - 1) - 1 : (2 * i) , (2 * i - 3) - 1 : (2 * i - 2)] = P2
        else:
            A[(2 * i - 1) - 1 : (2 * i) , (2 * i - 1) - 1 : (2 * i)]     = S1
            A[(2 * i - 1) - 1 : (2 * i) , (2 * i - 3) - 1 : (2 * i - 2)] = S2

    B = np.zeros((2 * n_vehicle, n_cav))
    for i in range(0, n_cav):
        B[pos_cav[i] * 2 + 1, i] = 1

    if measure_type == 1:
        C = np.zeros((n_vehicle, 2 * n_vehicle))
        for i in range(0, n_vehicle):
            C[i, 2 * i + 1] = 1
    elif measure_type == 2:
        C = np.zeros((2 * n_vehicle, 2 * n_vehicle))
        for i in range(0, n_vehicle):
            C[i, 2 * i + 1] = 1
            C[n_vehicle + i, 2 * i] = 1
    elif measure_type == 3:
        C = np.zeros((n_vehicle + n_cav, 2 * n_vehicle))
        for i in range(0, n_vehicle):
            C[i, 2 * i + 1] = 1
        for i in range(0, n_cav):
            C[n_vehicle + i, 2 * pos_cav[i]] = 1

    Ad = Ts * A + np.eye(2 * n_vehicle)
    Bd = B * Ts
    Cd = C

    return Ad, Bd, Cd

    
