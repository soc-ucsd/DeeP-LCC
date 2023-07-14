import numpy as np
import math
from scipy.io import savemat

'''
       Generate heterogeneous HDV paramters
'''

hdv_type = 2
v_star = 15

'''
      ID   = [0,0,1,0,0,1,0,0];    % ID of vehicle types
                                % 1: CAV  0: HDV
'''

# Homegeneous setup for OVM
alpha = 0.6 * np.ones((8, 1))
beta = 0.9 * np.ones((8,1))
s_go = 25 * np.ones((8,1))
s_st = 5
v_max = 30

# Equilibrium spacing
s_star = math.acos(1 - v_star / v_max * 2) / math.pi * (s_go - s_st) + s_st  # ./

hdv_parameter = {"type": hdv_type, "alpha": alpha, "beta": beta, "s_st": s_st, "s_go": s_go, "v_max": v_max, "s_star": s_star}
savemat('./_data/hdv_ovm_homogeneous.mat', hdv_parameter)

if hdv_type == 1:
    # Driver Model: OVM
    alpha = 0.4 + 0.4 * np.random.rand(8, 1)
    beta = 0.7 + 0.4 * np.random.rand(8, 1)
    s_go = 30 + 10 * np.random.rand(8, 1)
    s_st = 5
    v_max = 30

    # Manual set for parameters
    alpha = np.array([0.45, 0.75, 0.60, 0.70, 0.50, 0.60, 0.40, 0.80])
    beta = np.array([0.60, 0.95, 0.90, 0.95, 0.75, 0.90, 0.80, 1.00])
    s_go = np.array([38, 31, 35, 33, 37, 35, 39, 34])

    # Consider nominal parameter for the CAV position, which only works
    # in comparison for all the vehicles are HDVs
    alpha[2] = 0.6
    alpha[5] = 0.6
    beta[2] = 0.9
    beta[5] = 0.9
    s_go[2] = 35
    s_go[5] = 35

    # Equilibrium spacing
    s_star = np.dot((math.acos(1 - v_star / v_max * 2) / math.pi), (s_go - s_st)) + s_st

elif hdv_type == 2:
    # Driver Model: IDM
    v_max = 30
    T_gap = np.ones((8, 1))
    a = 1
    b = 1.5
    delta = 4
    s_st = 5

    # Equilibrium spacing
    s_star = (s_st + T_gap * v_star) / math.sqrt(1 - (v_star / v_max) ** delta)

if hdv_type == 1:
    hdv_parameter = {"type": hdv_type, "alpha": alpha, "beta": beta, "s_st": s_st, "s_go": s_go, "v_max": v_max,
                     "s_star": s_star}
    hdv_parameter_dict = {"hdv_parameter": hdv_parameter}
    savemat('./_data/hdv_ovm.mat', hdv_parameter_dict)
elif hdv_type == 2:
    hdv_parameter = {"type": hdv_type, "v_max": v_max, "T_gap": T_gap, "a": a, "b": b, "delta": delta, "s_st": s_st, "s_star": s_star}
    hdv_parameter_dict = {"hdv_parameter": hdv_parameter}
    savemat('./_data/hdv_idm.mat', hdv_parameter_dict)





