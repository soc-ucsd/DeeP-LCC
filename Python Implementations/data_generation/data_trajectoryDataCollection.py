import numpy as np
from scipy.io import loadmat
from scipy.io import savemat
import sys
sys.path.append('./_fcn')
import HDV_dynamics
import measure_mixed_traffic
import hankel_matrix

'''
      Data collection for random times
'''

data_total_number = 100

for i_data in range(data_total_number):

    '''
          Parameter setup
    '''

    # Type for HDV car-following model
    hdv_type = 1        # 1. OVM   2. IDM
    # Uncertainty for HDV behavior
    acel_noise = 0.1    # A white noise signal on HDV's original acceleration
    # Data set
    data_str = "3"      # 1. random ovm  2. manual ovm  3. homogeneous ovm

    # Parameters in Simulation
    total_time = 40     # Total Simulation Time
    Tstep = 0.05        # Time Step
    total_time_step = total_time / Tstep

    # DeePC Formulation
    T = 2000            # length of data samples
    Tini = 20           # length of past data
    N = 50              # length of predicted horizon

    weight_v = 1        # weight coefficient for velocity error
    weight_s = 0.5      # weight coefficient for spacing error
    weight_u = 0.1      # weight coefficient for control input

    lambda_g = 1        # penalty on ||g||_2^2 in objective
    lambda_y = 1e3      # penalty on ||sigma_y||_2^2 in objective

    # System Dynamics
    # vel_noise = 0.1;         # noise signal in velocity signal

    '''
        Parameters in Mixed Traffic
    '''
    ID = [0, 0, 1, 0, 0, 1, 0, 0]               # ID of vehicle types
                                                # 1: CAV  0: HDV
    pos_cav = np.argwhere(np.ravel(ID) == 1)    # position of CAVs
    n_vehicle = len(ID)                         # number of vehicles
    n_cav = len(pos_cav)                        # number of CAVs
    n_hdv = n_vehicle - n_cav                   # number of HDVs

    mix = 1                                     # whether mixed traffic flow

    v_star = 15                                 # Equilibrium velocity
    s_star = 20                                 # Equilibrium spacing for CAV

    if hdv_type == 1:
        # Driver Model: OVM
        file_path = './_data/hdv_ovm_' + data_str + '.mat'
        hdv_parameter_tmp = loadmat(file_path, mat_dtype=True)
        hdv_parameter = {"type": hdv_parameter_tmp["hdv_parameter"]["type"][0][0][0][0],
                         "alpha": hdv_parameter_tmp["hdv_parameter"]["alpha"][0][0],
                         "beta": hdv_parameter_tmp["hdv_parameter"]["beta"][0][0],
                         "s_st": hdv_parameter_tmp["hdv_parameter"]["s_st"][0][0][0][0],
                         "s_go": hdv_parameter_tmp["hdv_parameter"]["s_go"][0][0],
                         "v_max": hdv_parameter_tmp["hdv_parameter"]["v_max"][0][0][0][0],
                         "s_star": hdv_parameter_tmp["hdv_parameter"]["s_star"][0][0]}
    elif hdv_type == 2:
        # Driver Model: IDM
        hdv_parameter_tmp = loadmat('./_data/hdv_idm.mat', mat_dtype=True)
        hdv_parameter = {"type": hdv_parameter_tmp["hdv_parameter"]["type"],
                         "v_max": hdv_parameter_tmp["hdv_parameter"]["v_max"][0][0][0][0],
                         "T_gap": hdv_parameter_tmp["hdv_parameter"]["T_gap"][0][0],
                         "a": hdv_parameter_tmp["hdv_parameter"]["a"][0][0][0][0],
                         "b": hdv_parameter_tmp["hdv_parameter"]["b"][0][0][0][0],
                         "delta": hdv_parameter_tmp["hdv_parameter"]["delta"][0][0][0][0],
                         "s_st": hdv_parameter_tmp["hdv_parameter"]["s_st"][0][0][0][0],
                         "s_star": hdv_parameter_tmp["hdv_parameter"]["s_star"][0][0]}

    #         v_max   = 30;
    #         T_gap   = 1;
    #         a       = 1;
    #         b       = 1.5;
    #         delta   = 4;
    #         s_st    = 5;
    #         % Equilibrium spacing
    #         s_star  = (s_st+T_gap*v_star)/sqrt(1-(v_star/v_max)^delta);

    acel_max = 2
    dcel_max = -5

    # What is measurable
    # for measure_type = 2:3
    measure_type = 3
    # 1. Only the velocity errors of all the vehicles are measurable;
    # 2. All the states, including velocity error and spacing error are measurable;
    # 3. Velocity error and spacing error of the CAVs are measurable,
    #    and the velocity error of the HDVs are measurable.

    '''
        size in DeePC
    '''

    n_ctr = 2 * n_vehicle       # number of state variables
    m_ctr = n_cav               # number of input variables
    if measure_type == 1:       # number of output variables
        p_ctr = n_vehicle
    elif measure_type == 2:
        p_ctr = 2 * n_vehicle
    elif measure_type == 3:
        p_ctr = n_vehicle + n_cav

    '''
        Scenario initialization
    '''

    # There is one head vehicle at the very beginning
    # S = np.zeros((int(total_time_step), n_vehicle + 1, 3))
    S = np.zeros((T, n_vehicle + 1, 3))
    S[0, 0, 0] = 0
    for i in range(1, n_vehicle + 1):
        S[0, i, 0] = S[0, i - 1, 0] - hdv_parameter["s_star"][i - 1]
    S[0, :, 1] = np.dot(v_star, np.ones((1, n_vehicle + 1)))

    '''
    Data collection
    '''

    # persistently exciting input data
    
    #test1 = np.ones((m_ctr, T));
    #for k in range(0, m_ctr):
    #    for x in range(0,T):
    #        test1[k, x] = (k+1)*(x+1)+(x+1);

    #test2 = np.ones((1,T));
    #for x in range(0,T):
    #    test2[0, x] = x+1+x+1;

    ud = -1 + 2 * np.random.rand(m_ctr, T)
    #ud = -1 + np.dot(2, test1);
    ed = -1 + 2 * np.random.rand(1, T)
    #ed = -1 + np.dot(2, test2);
    yd = np.zeros((p_ctr, T))

    #test3 = np.ones((n_vehicle,1));
    #for k in range(0,n_vehicle):
    #    test3[k, 0] = (k+1)*(k+1);
    # generate output data
    for k in range(0, T - 1):
        # Update acceleration
        acel = HDV_dynamics.HDV_dynamics(S[k, :, :], hdv_parameter) - acel_noise + 2 * acel_noise * np.random.rand(n_vehicle, 1)
        #acel = HDV_dynamics.HDV_dynamics(S[k, :, :], hdv_parameter) - acel_noise + np.dot((2 * acel_noise), test3)

        S[k, 0, 2] = 0                                # the head vehicle
        S[k, 1:, 2] = acel.reshape((8,))              # all the vehicles using HDV model
        S[k, pos_cav+1, 2] = ud[:, k].reshape((2, 1))   # the CAVs

        S[k + 1, :, 1] = S[k, :, 1] + Tstep * S[k, :, 2]
        S[k + 1, 0, 1] = ed[0][k] + v_star            # the velocity of the head vehicle
        S[k + 1, :, 0] = S[k, :, 0] + Tstep * S[k, :, 1]

        yd[:, k] = measure_mixed_traffic.measure_mixed_traffic(S[k, 1:, 1], S[k, :, 0], ID, v_star, s_star, measure_type).reshape((10,))

    k = k + 1
    yd[:, k] = measure_mixed_traffic.measure_mixed_traffic(S[k, 1:, 1], S[k, :, 0], ID, v_star, s_star, measure_type).reshape((10,))

    # organize past data and future data
    U = hankel_matrix.hankel_matrix(ud, Tini + N)
    Up = U[:Tini * m_ctr, :]
    Uf = U[(Tini * m_ctr + 1) - 1:, :]

    E = hankel_matrix.hankel_matrix(ed, Tini + N)
    Ep = E[: Tini, :]
    Ef = E[(Tini + 1) - 1:, :]

    Y = hankel_matrix.hankel_matrix(yd, Tini + N)
    Yp = Y[: Tini * p_ctr, :]
    Yf = Y[(Tini * p_ctr + 1) - 1:, :]

    print("Processing..."+ str(i_data / data_total_number * 100)+"%")
    # print(S[:, :, 0])

    mat_dict = {"hdv_type": hdv_type, "acel_noise": acel_noise, "Up": Up, "Yp": Yp, "Uf": Uf, "Yf": Yf, "Ep": Ep,
                "Ef": Ef, "T": T, "Tini": Tini, "N": N, "ID": ID, "Tstep": Tstep, "v_star": v_star}
    mat_path = "./_data/trajectory_data_collection/data" + data_str + "_" + str(i_data) + "_noiseLevel_" + str(acel_noise) + ".mat"
    savemat(mat_path, mat_dict)
