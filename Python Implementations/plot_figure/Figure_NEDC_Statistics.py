import numpy as np
from scipy.io import loadmat
from scipy.ndimage.filters import uniform_filter1d

def smooth(a,WSZ):
    # a: NumPy 1-D array containing the data to be smoothed
    # WSZ: smoothing window size needs, which must be odd number,
    # as in the original MATLAB implementation
    out0 = np.convolve(a,np.ones(WSZ,dtype=int),'valid')/WSZ    
    r = np.arange(1,WSZ-1,2)
    start = np.cumsum(a[:WSZ-1])[::2]/r
    stop = (np.cumsum(a[:-WSZ:-1])[::2]/r)[::-1]
    return np.concatenate((  start , out0, stop  ))


# Data set configuration
data_str        = '2'  # 1. random ovm  2. manual ovm  3. homogeneous ovm
hdv_type        = 1    # 1. OVM   2. IDM
acel_noise      = 0.1  # A white noise signal on HDV's original acceleration

# Head vehicle trajectory
trajectory_id = '1'
head_vehicle_trajectory = loadmat('./_data/nedc_modified_v' + trajectory_id + '.mat')
end_time = head_vehicle_trajectory['time'][-1]

initialization_time = 30               # Time for the original HDV-all system to stabilize
adaption_time       = 20               # Time for the CAVs to adjust to their desired state

# Total simulation results
# total_time          = initialization_time + adaption_time + end_time  # Total Simulation Time
# begin_time          = initialization_time + adaption_time

# Seperate different phases in the simulation
# time_point = [60,88,121,176,206]  # For Trajectory V1
time_point = [60,88,121,166,196]   # For Trajectory V2
time_i     = 3
begin_time = time_point[time_i]
total_time = time_point[time_i+1]

weight_v     = 1        # weight coefficient for velocity error
weight_s     = 0.5      # weight coefficient for spacing error   
weight_u     = 0.1      # weight coefficient for control input

lambda_g     = 100      # penalty on ||g||_2^2 in objective
lambda_y     = 10000      # penalty on ||sigma_y||_2^2 in objective

i_data          = 2

S_MPC_dic      = loadmat('_data/simulation_data/MPC/nedc_simulation/simulation_data' + data_str + '_modified_v' + trajectory_id + '_noiseLevel_' + str(acel_noise) +
                     '_hdvType_' + str(hdv_type) + '.mat')

ID = S_MPC_dic["ID"][0]
Tstep = S_MPC_dic["Tstep"][0][0]
S_MPC = S_MPC_dic["S"]
S_DeePC_dic    = loadmat('_data/simulation_data/DeeP_LCC/nedc_simulation/simulation_data' + data_str + '_' + str(i_data) + '_modified_v' + trajectory_id + '_noiseLevel_' + str(acel_noise) +
                     '_hdvType_' + str(hdv_type) + '_lambdaG_' + str(lambda_g) + '_lambdaY_' + str(lambda_y) + '.mat')
S_DeePC = S_DeePC_dic["S"]

S_HDV_dic      = loadmat('_data/simulation_data/HDV/nedc_simulation/simulation_data' + data_str + '_modified_v' + trajectory_id + '_noiseLevel_' + str(acel_noise) +
                     '_hdvType_' + str(hdv_type) + '.mat')
S_HDV = S_HDV_dic["S"]

n_vehicle   = len(ID)           # number of vehicles
# Smooth acceleration signal
smooth_window = 9
for i in range(1, n_vehicle+1):
    S_DeePC[:, i, 2] = smooth(S_DeePC[:, i, 2], smooth_window)
    S_MPC[:, i, 2] = smooth(S_MPC[:, i, 2], smooth_window)
    S_HDV[:, i, 2] = smooth(S_HDV[:, i, 2], smooth_window)
    
    

FuelConsumption = np.zeros(3)
VelocityError   = np.zeros(3)

for i in range(int(begin_time//Tstep), int(total_time//Tstep)):
    R_DeePC  = 0.333 + 0.00108 * S_DeePC[i, 3:, 1] ** 2 + 1.2 * S_DeePC[i, 3:, 2]
    F_DeePC  = 0.444 + 0.09 * R_DeePC * S_DeePC[i, 3:, 1] + 0.054 * np.maximum(0, S_DeePC[i, 3:, 2]) ** 2 * S_DeePC[i, 3:, 1]
    F_DeePC[R_DeePC <= 0] = 0.444
    FuelConsumption[0] += np.sum(F_DeePC) * Tstep

    VelocityError[0] += np.sum(np.abs(S_DeePC[i, 3:, 1] - S_DeePC[i, 0, 1]) / S_DeePC[i, 0, 1])

    R_MPC  = 0.333 + 0.00108 * S_MPC[i, 3:, 1] ** 2 + 1.2 * S_MPC[i, 3:, 2]
    F_MPC  = 0.444 + 0.09 * R_MPC * S_MPC[i, 3:, 1] + 0.054 * np.maximum(0, S_MPC[i, 3:, 2]) ** 2 * S_MPC[i, 3:, 1]
    F_MPC[R_MPC <= 0] = 0.444
    FuelConsumption[1] += np.sum(F_MPC) * Tstep

    VelocityError[1] += np.sum(np.abs(S_MPC[i, 3:, 1] - S_MPC[i, 0, 1]) / S_MPC[i, 0, 1])

    R_HDV  = 0.333 + 0.00108 * S_HDV[i, 3:, 1] ** 2 + 1.2 * S_HDV[i, 3:, 2]
    F_HDV  = 0.444 + 0.09 * R_HDV * S_HDV[i, 3:, 1] + 0.054 * np.maximum(0, S_HDV[i, 3:, 2]) ** 2 * S_HDV[i, 3:, 1]
    F_HDV[R_HDV <= 0] = 0.444
    FuelConsumption[2] += np.sum(F_HDV) * Tstep

    VelocityError[2] += np.sum(np.abs(S_HDV[i, 3:, 1] - S_HDV[i, 0, 1]) / S_HDV[i, 0, 1])

VelocityError = VelocityError /n_vehicle / ((total_time - begin_time) / Tstep)

FuelComparison = (FuelConsumption[2] - FuelConsumption) / FuelConsumption[2] * 100
VelocityComparison = (VelocityError[2] - VelocityError) / VelocityError[2] * 100


print('Fuel Consumption:   DeePC  |    MPC    |   HDVs')
print('                  {:.4f}  |  {:.4f}  |   {:.4f}'.format(FuelConsumption[0], FuelConsumption[1], FuelConsumption[2]))
print('                  {:.2f}%  |  {:.2f}%  |   {:.2f}%'.format(FuelConsumption[0], FuelComparison[1], FuelComparison[2]))
print('Velocity Error:   DeePC  |    MPC    |   HDVs')
print('                  {:.4f}  |  {:.4f}  |   {:.4f}'.format(VelocityError[0], VelocityError[1], VelocityError[2]))
print('                  {:.2f}%  |  {:.2f}%  |   {:.2f}%'.format(VelocityComparison[0], VelocityComparison[1], VelocityComparison[2]))

