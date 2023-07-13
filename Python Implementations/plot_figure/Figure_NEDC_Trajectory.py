from scipy.io import loadmat
import numpy as np
import matplotlib.pyplot as plt

'''
    Analysis for simulation results under same data sample
'''

# Data set
data_str        = '2'  # 1. random ovm  2. manual ovm  3. homogeneous ovm
# Mix or not
mix             = 1    # 0. all HDVs; 1. mix
# Type of the controller
controller_type = 2    # 1. DeeP-LCC  2. MPC
# Type for HDV car-following model
hdv_type        = 1    # 1. OVM   2. IDM
# Uncertainty for HDV behavior
acel_noise      = 0.1  # A white noise signal on HDV's original acceleration

# Head vehicle trajectory
trajectory_id = '1'
head_vehicle_trajectory_tmp = loadmat('../_data/nedc_modified_v'+str(trajectory_id)+'.mat')
head_vehicle_trajectory        = {"time": head_vehicle_trajectory_tmp["time"][0], "vel": head_vehicle_trajectory_tmp["vel"]}
end_time = head_vehicle_trajectory["time"][-1]

initialization_time = 30  # Time for the original HDV-all system to stabilize
adaption_time       = 20  # Time for the CAVs to adjust to their desired state
total_time          = int(initialization_time + adaption_time + end_time)  # Total Simulation Time
begin_time          = initialization_time + adaption_time

weight_v     = 1    # weight coefficient for velocity error
weight_s     = 0.5  # weight coefficient for spacing error
weight_u     = 0.1  # weight coefficient for control input

lambda_g     = 100  # penalty on ||g||_2^2 in objective
lambda_y     = 1e4  # penalty on ||sigma_y||_2^2 in objective

i_data          = 1

if mix:
    if controller_type == 1:
        tmp_mat = loadmat('../_data/simulation_data/DeeP_LCC/nedc_simulation/simulation_data'+data_str+'_'+str(i_data)+'_modified_v'+str(trajectory_id)+'_noiseLevel_'+str(acel_noise)+ '_hdvType_'+str(hdv_type)+'_lambdaG_'+str(lambda_g)+'_lambdaY_'+str(lambda_y)+'.mat')
        data_dict = {"ID": tmp_mat["ID"][0], "N": tmp_mat["N"][0][0], "S": tmp_mat["S"], "T": tmp_mat["T"][0][0], "Tini": tmp_mat["Tini"][0][0], "Tstep": tmp_mat["Tstep"][0][0], "acel_noise": tmp_mat["acel_noise"][0][0], "hdv_type": tmp_mat["hdv_type"][0][0], "v_star": tmp_mat["v_star"][0][0]}
        controller_str = 'DeePC'
    elif controller_type == 2:
        tmp_mat = loadmat('../_data/simulation_data/MPC/nedc_simulation/simulation_data'+data_str+'_modified_v'+str(trajectory_id)+'_noiseLevel_'+str(acel_noise)+'_hdvType_'+str(hdv_type)+'.mat')
        data_dict = {"ID": tmp_mat["ID"][0], "N": tmp_mat["N"][0][0], "S": tmp_mat["S"],
                     "T": tmp_mat["T"][0][0], "Tini": tmp_mat["Tini"][0][0],
                     "Tstep": tmp_mat["Tstep"][0][0], "acel_noise": tmp_mat["acel_noise"][0][0],
                     "hdv_type": tmp_mat["hdv_type"][0][0], "v_star": tmp_mat["v_star"][0][0]}
        controller_str = 'MPC'
else:  # ngsim simulation
    tmp_mat = loadmat('../_data/simulation_data/HDV/nedc_simulation/simulation_data'+data_str+'_modified_v'+str(trajectory_id)+'_noiseLevel_'+str(acel_noise)+'_hdvType_'+str(hdv_type)+'.mat')
    data_dict = {"ID": tmp_mat["ID"][0], "N": tmp_mat["N"][0][0], "S": tmp_mat["S"],
                 "T": tmp_mat["T"][0][0], "Tini": tmp_mat["Tini"][0][0],
                 "Tstep": tmp_mat["Tstep"][0][0], "acel_noise": tmp_mat["acel_noise"][0][0],
                 "hdv_type": tmp_mat["hdv_type"][0][0], "v_star": tmp_mat["v_star"][0][0]}
    controller_str = 'HDV'

ID = data_dict["ID"]
S = data_dict["S"]
Tstep = data_dict["Tstep"]

n_vehicle   = len(ID)  # number of vehicles

''''
       Plot Results
'''

color_gray  = [i//255 for i in [170, 170, 170]]
color_red   = [i//255 for i in [244, 53, 124]]
color_blue  = [i//255 for i in [67, 121, 227]]
color_black = [0, 0, 0]
color_orange = [i//255 for i in [255,132,31]]
color_blue_2 = [i//255 for i in [61, 90, 128]]
color_red_2  = [i//255 for i in [238, 108, 77]]
label_size  = 18
total_size  = 14
line_width  = 1.5

# Head vehicle trajectory
time_point = [60,88,121,176,206]  # For Trajectory V1
# time_point = [60,88,121,166,196]   # For Trajectory V2
time_scale = [i/100-(initialization_time + adaption_time) for i in range(begin_time*100, total_time*100+1, int(Tstep*100))]

plt.plot(time_scale, S[int(begin_time // Tstep): int(total_time // Tstep)+1, 0, 1], "k")
plt.xlim([begin_time - (initialization_time + adaption_time), total_time- (initialization_time + adaption_time)])
plt.ylim([10, 30])

for time_i in range(4):
    plt.plot(time_point[time_i] * np.ones((20, 1)) - (initialization_time + adaption_time), np.linspace(10, S[int(time_point[time_i] // Tstep), 0, 1], 20), "--", "b")
    plt.text((time_point[time_i] + time_point[time_i+1]) // 2 - (initialization_time + adaption_time), 11.5, 'phase ' + str(time_i+1), horizontalalignment = "center", color = "r")

plt.xlabel('$t$ [$\mathrm{s}$]', color = "k")
plt.ylabel('Velocity [$\mathrm{m/s}$]', color = "k")
plt.yticks(list(range(10,31,5)))
plt.grid()
plt.savefig("./1.png")
plt.show()

# print('.\figs\NEDC_HeadVehicleTrajectory'+'-painters'+'-depsc2'+'-r300')

# Velocity
# for time_i in range(4):
#     begin_time = time_point[time_i]
#     total_time = time_point[time_i + 1]
# figure;
# plt.plot(list(range(int(begin_time), int(total_time), int(Tstep))), S[int(begin_time // Tstep(int(total_time // Tstep) + 1, 0, 1], "c")

# for i in range(n_vehicle):
#     if ID(i) == 1:
    #     plt.plot(list(range(int(begin_time), int(total_time), int(Tstep))), S[int(begin_time // Tstep(int(total_time // Tstep) + 1, 0, 1], "r")   # line for velocity of CAVs
# else:
#     plt.plot(list(range(int(begin_time), int(total_time), int(Tstep))), S[int(begin_time // Tstep(int(total_time // Tstep) + 1, 0, 1], "r")  # line for velocity of HDVs
# plt.xlim([begin_time, total_time])
# plt.xlabel('$t$ [$\mathrm{s}$]', 'k')
# plt.ylabel('Velocity [$\mathrm{m/s}$]', 'k')

total_time = initialization_time + adaption_time + end_time # Total Simulation Time
begin_time = initialization_time + adaption_time

plt.plot(time_scale, S[int(begin_time // Tstep): int(total_time // Tstep)+1, 0, 1], "k")
for i in range(n_vehicle):
    if ID[i] == 0:
        plt.plot(time_scale, S[int(begin_time // Tstep): int(total_time // Tstep)+1, i+1, 1], "c")
id_cav = 1
for i in range(n_vehicle):
    if ID[i] == 1:
        if id_cav == 1:
            plt.plot(time_scale, S[int(begin_time // Tstep): int(total_time // Tstep)+1, i+1, 1], "r")
            id_cav = id_cav +1
        elif id_cav == 2:
            plt.plot(time_scale, S[int(begin_time // Tstep): int(total_time // Tstep)+1, i + 1, 1], "b")

for time_i in range(4):
    plt.plot(time_point[time_i] * np.ones((20, 1)) - (initialization_time + adaption_time), np.linspace(10, S[int(time_point[time_i] // Tstep), 0, 1], 20), "--", "b")
    plt.text((time_point[time_i] + time_point[time_i+1]) // 2 - (initialization_time + adaption_time), 12.5, 'phase ' + str(time_i+1), horizontalalignment = "center", color = "r")
plt.xlim([time_scale[0], time_scale[-1]])
plt.ylim([11.5, 28.5])
plt.xlabel('$t$ [$\mathrm{s}$]', color = "k")
plt.ylabel('Velocity [$\mathrm{m/s}$]', color = "k")
plt.grid()
plt.savefig("./2.png")
plt.show()

# begin_time_1 = 65
# total_time_1 = 75
#
# plt.plot(list(range(begin_time_1, total_time_1, Tstep)), S[int(begin_time_1 // Tstep): int(total_time_1 // Tstep) + 1, 0, 1], "k")  # line for velocity of HDVs
# for i in range(n_vehicle):
#     if ID[i] == 0:
#         plt.plot(list(range(begin_time_1, total_time_1, Tstep)), S[int(begin_time_1 // Tstep): int(total_time_1 // Tstep) + 1, i+1, 1], "c")  # line for velocity of HDVs
# id_cav = 1;
# for i in range(n_vehicle):
#     if ID[i] == 1:
#         if id_cav == 1:
#             plt.plot(list(range(begin_time_1, total_time_1, Tstep)), S[int(begin_time_1 // Tstep): int(total_time_1 // Tstep) + 1, i+1, 1], "r")  # line for velocity of CAVs
# id_cav = id_cav + 1
# elif:
#     id_cav == 2
#     plt.plot(list(range(begin_time_1, total_time_1, Tstep)), S[int(begin_time_1 // Tstep): int(total_time_1 // Tstep) + 1, i+1, 1], "b")  # line for velocity of CAVs
# plt.ylim([13, 18])
#
# if mix:
#     print('.\figs\NEDC_VelocityProfile_Controller_'+ str(controller_type) + '-painters' + '-depsc2' + '-r300')
# else:
#     print('.\figs\NEDC_VelocityProfile_AllHDVs' + '-painters' + '-depsc2' + '-r300')


# Spacing
for i in range(n_vehicle):
    if ID[i] == 0:
        plt.plot(time_scale, S[int(begin_time // Tstep): int(total_time // Tstep)+1, i, 0] - S[int(begin_time // Tstep): int(total_time // Tstep)+1, i+1, 0], "c")
id_cav = 1
for i in range(n_vehicle):
    if ID[i] == 1:
        if id_cav == 1:
            plt.plot(time_scale, S[int(begin_time // Tstep): int(total_time // Tstep)+1, i, 0] - S[int(begin_time // Tstep): int(total_time // Tstep)+1, i+1, 0], "r")
            id_cav = id_cav +1
        elif id_cav == 2:
            plt.plot(time_scale, S[int(begin_time // Tstep): int(total_time // Tstep)+1, i, 0] - S[int(begin_time // Tstep): int(total_time // Tstep)+1, i+1, 0], "b")

plt.xlim([time_scale[0], time_scale[-1]])
plt.xlabel('$t$ [$\mathrm{s}$]', color = "k")
plt.ylabel('Spacing [$\mathrm{m}$]', color = "k")
plt.grid()
plt.savefig("./3.png")
plt.show()

# if mix:
#   print('.\figs\NGSIM_SpacingProfile_Controller_'+str(controller_type)+'-painters'+'-depsc2'+'-r300')
# else:
#   print('.\figs\NGSIM_SpacingProfile_AllHDVs'+'-painters'+'-depsc2'+'-r300')




# acceleration
def smooth(a,WSZ):
    # a: NumPy 1-D array containing the data to be smoothed
    # WSZ: smoothing window size needs, which must be odd number,
    # as in the original MATLAB implementation
    out0 = np.convolve(a,np.ones(WSZ,dtype=int),'valid')/WSZ
    r = np.arange(1,WSZ-1,2)
    start = np.cumsum(a[:WSZ-1])[::2]/r
    stop = (np.cumsum(a[:-WSZ:-1])[::2]/r)[::-1]
    return np.concatenate((  start , out0, stop))

for i in range(n_vehicle):
    # smooth acceleration signal
    S[:, i + 1, 2] = smooth(S[:, i + 1, 2], 9)
for i in range(n_vehicle):
    if ID[i] == 0:
        plt.plot(time_scale, S[int(begin_time // Tstep): int(total_time // Tstep)+1, i+1, 2], "c")
id_cav = 1
for i in range(n_vehicle):
    if ID[i] == 1:
        if id_cav == 1:
            plt.plot(time_scale, S[int(begin_time // Tstep): int(total_time // Tstep)+1, i+1, 2], "r")
            id_cav = id_cav +1
        elif id_cav == 2:
            plt.plot(time_scale, S[int(begin_time // Tstep): int(total_time // Tstep)+1, i+1, 2], "b")
plt.xlim([time_scale[0], time_scale[-1]])
plt.xlabel('$t$ [$\mathrm{s}$]', color = "k")
plt.ylabel('Acceleration [$\mathrm{m/s^2}$]', color = "k")
plt.grid()
plt.savefig("./4.png")
plt.show()

# -------------------------------------------------------------------------
# Calculate Performance Indexes
# --------------------------------------------------------------------------
FuelConsumption = 0
VelocityError = 0
for i in range(int(begin_time // Tstep), int(total_time // Tstep)+1):
    R = 0.333 + 0.00108 * S[i, 3:, 1] ** 2 + 1.2 * S[i, 3:, 2]
    Fuel = 0.444 + 0.09 * R * S[i, 3:, 1] + 0.054 * max(0, S[i, 3:, 2].all()) ** 2. * S[i, 3:, 1]
    Fuel[R <= 0] = 0.444
    FuelConsumption = FuelConsumption + sum(Fuel) * Tstep
    VelocityError = VelocityError + sum(abs(S[i, 3:, 1]-S[i, 0, 1]) / S[i, 0, 1])

VelocityError = VelocityError / n_vehicle / ((total_time - begin_time) / Tstep)

print('Fuel comsumption:   ' + str(FuelConsumption))
print('Velocity error:   ' + str(VelocityError))

