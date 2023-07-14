import numpy as np

# =========================================================================
#               Calculate the acceleration of HDVs
# S:            state of all the vehicles
# type:         type of the HDV car-following model
# parameter:    Parameter value in the car-following model
# =========================================================================

def HDV_dynamics(S, parameter):

    num_vehicle = S.shape[0] - 1

    acel_max    = 2
    dcel_max    = -5

    if parameter["type"] == 1:
        V_diff = []
        D_diff = []
        for i in range(S.shape[0]-1):
            V_diff.append(S[i][1] - S[i+1][1])
            D_diff.append(S[i][0] - S[i+1][0])
        V_diff = np.array(V_diff)
        D_diff = np.array(D_diff)
        #V_diff = S[:-1, 1] - S[1:, 1]
        #D_diff = S[:-1, 0] - S[1:, 0]
        cal_D = np.transpose([D_diff])
        for i in range(num_vehicle):
            if cal_D[i] > parameter["s_go"][i]:
                cal_D[i] = parameter["s_go"][i]
            elif cal_D[i] < parameter["s_st"]:
                cal_D[i] = parameter["s_st"]
        
        # print(parameter['beta'].shape)
        # print(V_diff.T.shape)


        acel = parameter['alpha'] * (parameter['v_max'] / 2 * (1 - np.cos(np.pi * (cal_D.reshape((8,1)) - parameter['s_st']) / (parameter['s_go']
                                                            - parameter['s_st']))) - S[1:, 1].T.reshape((8,1))) + parameter['beta'] * V_diff.T.reshape((8,1))
        
        acel[acel > acel_max] = acel_max
        acel[acel < dcel_max] = dcel_max
        
        acel_sd = (S[1:, 1]**2 - S[: -1, 1]**2) / 2 / D_diff
        acel[acel_sd > abs(dcel_max)] = dcel_max
        
    elif parameter["type"] ==2:
        v_max       = 30
        T_gap       = 1
        a           = 1
        b           = 1.5
        delta       = 4
        s_st        = 5
        
        V_diff = S[0:(S.shape[0]-1), 1] - S[1:S.shape[0], 1]
        D_diff = S[0:(S.shape[0]-1), 0] - S[1:S.shape[0], 0]
        
        acel = a * (1 - (S[1:, 1] / v_max) ** delta -
           ((s_st + T_gap * S[1:, 1] - V_diff * S[1:, 1] / 2 / np.sqrt(a) / np.sqrt(b)) / D_diff) ** 2)
        acel = np.transpose([acel])
        
        acel_sd = (S[1:, 1]**2 - S[:-1, 1]**2) / 2 / D_diff
        acel[acel_sd > abs(dcel_max)] = dcel_max
        
    return acel