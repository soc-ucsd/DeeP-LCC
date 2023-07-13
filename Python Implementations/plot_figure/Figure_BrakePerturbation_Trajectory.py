# =========================================================================
#               Analysis for simulation results under same data sample               
# =========================================================================
import numpy as np
from scipy.io import loadmat
from scipy.ndimage import uniform_filter1d
import matplotlib.pyplot as plt

def smooth(a,WSZ):
    # a: NumPy 1-D array containing the data to be smoothed
    # WSZ: smoothing window size needs, which must be odd number,
    # as in the original MATLAB implementation
    out0 = np.convolve(a,np.ones(WSZ,dtype=int),'valid')/WSZ    
    r = np.arange(1,WSZ-1,2)
    start = np.cumsum(a[:WSZ-1])[::2]/r
    stop = (np.cumsum(a[:-WSZ:-1])[::2]/r)[::-1]
    return np.concatenate((start , out0, stop  ))

# Data set
data_str         = '2';  # 1. random ovm  2. manual ovm  3. homogeneous ovm
# Mix or not
mix              = 0;    # 0. all HDVs; 1. mix
# Type of the controller
controller_type  = 1;    # 1. DeeP-LCC  2. MPC 
# Type for HDV car-following model
hdv_type         = 1;    # 1. OVM   2. IDM
# Uncertainty for HDV behavior
acel_noise       = 0.1;  # A white noise signal on HDV's original acceleration
# Perturbation amplitude
per_type         = 2;   # 1. sinuoid perturbation 2. brake perturbation 3. ngsim simulation
per_amp          = 5;
# whether there exists constraints
constraint_bool  = 1;
# wheter fix equilibrium spacing
fixed_spacing_bool = 0;

# Simulation Time
begin_time       = 0.05;
end_time         = 40;              

i_data           = 1;     # Data set number

weight_v     = 1;        # weight coefficient for velocity error
weight_s     = 0.5;      # weight coefficient for spacing error   
weight_u     = 0.1;      # weight coefficient for control input

lambda_g     = 100;      # penalty on ||g||_2^2 in objective
lambda_y     = int(1e4);      # penalty on ||sigma_y||_2^2 in objective

if mix:
    if controller_type == 1:
        simData = loadmat('_data\simulation_data\DeeP_LCC\constrained_simulation\simulation_data'+str(data_str)+'_'+str(i_data)+'_perType_'+str(per_type)+'_noiseLevel_'+str(acel_noise)+'_fixSpacing_'+str(fixed_spacing_bool)+'_hdvType_'+str(hdv_type)+'_lambdaG_'+str(lambda_g)+'_lambdaY_'+str(lambda_y)+'.mat');
        controller_str = 'DeeP-LCC';
    else:
        simData = loadmat('_data\simulation_data\MPC\constrained_simulation\simulation_data'+str(data_str)+'_'+str(i_data)+'_perType_'+str(per_type)+'_noiseLevel_'+str(acel_noise)+'_fixSpacing_'+str(fixed_spacing_bool)+'_hdvType_'+str(hdv_type)+'.mat');
        controller_str = 'MPC';   
    simDataDict = {"ID": simData["ID"],
                    "Tstep": simData["Tstep"][0][0],
                    "S": simData["S"],
                    "pr_status": simData["pr_status"]};
else:
    if constraint_bool:
        simData = loadmat('_data\simulation_data\HDV\constrained_simulation\simulation_data'+str(data_str)+'_'+str(i_data)+'_perType_'+str(per_type)+'_noiseLevel_'+str(acel_noise)+'_hdvType_'+str(hdv_type)+'.mat');
        controller_str = 'DeeP-LCC';
    else:
        simData = loadmat('_data\simulation_data\HDV\simulation_data'+str(data_str)+'_'+str(i_data)+'_perType_'+str(per_type)+'_noiseLevel_'+str(acel_noise)+'_hdvType_'+str(hdv_type)+'_lambdaG_'+str(lambda_g)+'_lambdaY_'+str(lambda_y)+'.mat');
        controller_str = 'DeeP-LCC';
    simDataDict = {"ID": simData["ID"],
                    "Tstep": simData["Tstep"][0][0],
                    "S": simData["S"]};

n_vehicle   = simDataDict["ID"].shape[1];           # number of vehicles


# -------------------------------------------------------------------------
#   Plot Results
#--------------------------------------------------------------------------
color_gray  = np.array((190,190,190))/255;
color_red   = np.array((244, 53, 124))/255;
color_blue  = np.array((67, 121, 227))/255;
color_black = np.array((0,0,0));
color_orange = np.array((255,132,31))/255;
label_size  = 18;
total_size  = 14;
line_width  = 2;

# Velocity
fig1, ax1 = plt.subplots();
id_cav = 1;
time = np.arange(begin_time, end_time+0.01, simDataDict["Tstep"]);
ax1.plot(time, simDataDict["S"][int(begin_time/simDataDict["Tstep"]-1):int(end_time/simDataDict["Tstep"]),0,1], color='black', linewidth=(line_width-0.5));

for i in range(0, n_vehicle):
    if simDataDict["ID"][0, i] == 0:
        time = np.arange(begin_time, end_time+0.01, simDataDict["Tstep"]);
        ax1.plot(time, simDataDict["S"][int(begin_time/simDataDict["Tstep"]-1):int(end_time/simDataDict["Tstep"]),i+1,1],color='gray', linewidth=(line_width-0.5)); # line for velocity of HDVs
for i in range(0, n_vehicle):
    if simDataDict["ID"][0, i] == 1:
        if id_cav == 1:
            time = np.arange(begin_time, end_time+0.01, simDataDict["Tstep"]);
            ax1.plot(time, simDataDict["S"][int(begin_time/simDataDict["Tstep"]-1):int(end_time/simDataDict["Tstep"]),i+1,1], color='red', linewidth=line_width); # line for velocity of CAVs
            id_cav  = id_cav+1;
        elif id_cav == 2:
            time = np.arange(begin_time, end_time+0.01, simDataDict["Tstep"]);
            ax1.plot(time, simDataDict["S"][int(begin_time/simDataDict["Tstep"]-1):int(end_time/simDataDict["Tstep"]),i+1,1],color='blue', linewidth=line_width); # line for velocity of CAVs
        else:
            continue;
ax1.grid(True);
fig1.gca().tick_params(labelsize=total_size);
fig1.gca().ticklabel_format(useMathText=True);
ax1.set_ylim(0, 20);
ax1.set_xlim(0, 30);

ax1.set_xlabel('$t$ [$\mathrm{s}$]',fontsize=label_size, color='black');
ax1.set_ylabel('Velocity [$\mathrm{m/s}$]',fontsize=label_size, color='black');



# if mix
#     print(gcf,['.\figs\BrakePerturbation_VelocityProfile_Controller_',num2str(controller_type)],'-painters','-depsc2','-r300');
# else
#     print(gcf,'.\figs\BrakePerturbation_VelocityProfile_AllHDVs','-painters','-depsc2','-r300');
# end

# Spacing
fig2, ax2 = plt.subplots();
id_cav = 1;
for i in range(0, n_vehicle):
   if simData["ID"][0, i] == 1:
        if id_cav ==1:
            time = np.arange(begin_time, end_time+0.01, simDataDict["Tstep"]);
            ax2.plot(time, simDataDict["S"][int(begin_time/simDataDict["Tstep"]-1):int(end_time/simDataDict["Tstep"]), i, 0]-simDataDict["S"][int(begin_time/simDataDict["Tstep"]-1):int(end_time/simDataDict["Tstep"]), i+1, 0], color='red', linewidth=line_width); # line for velocity of CAVs
            id_cav = id_cav + 1;
        else: #if id_cav == 2
            time = np.arange(begin_time, end_time+0.01, simDataDict["Tstep"]);
            ax2.plot(time, simDataDict["S"][int(begin_time/simDataDict["Tstep"]-1):int(end_time/simDataDict["Tstep"]),i, 0]-simDataDict["S"][int(begin_time/simDataDict["Tstep"]-1):int(end_time/simDataDict["Tstep"]),i+1,0], color='blue', linewidth=line_width); # line for velocity of CAVs
# if mix
#     plot(begin_time:Tstep:end_time,5*ones(round((end_time-begin_time)/Tstep)+1),'--k','linewidth',line_width);
#     text(11,6.2,'$s_\mathrm{min}=5\,\mathrm{m}$','Interpreter','latex','FontSize',label_size);
# end
ax2.grid(True);
fig2.gca().tick_params(labelsize=total_size);
fig2.gca().ticklabel_format(useMathText=True);
ax2.set_ylim(5, 35);
ax2.set_xlim(0, 30);
ax2.set_yticks(range(5, 36, 10));

ax2.set_xlabel('$t$ [$\mathrm{s}$]', fontsize=label_size, color='black');
ax2.set_ylabel('Spacing [$\mathrm{m}$]', fontsize=label_size, color='black');

# if mix
#     print(gcf,['.\figs\BrakePerturbation_SpacingProfile_Controller_',num2str(controller_type)],'-painters','-depsc2','-r300');
# else
#     print(gcf,'.\figs\BrakePerturbation_SpacingProfile_AllHDVs','-painters','-depsc2','-r300');
# end

# Problem status
if mix:
    fig3, ax3 = plt.subplots();
    time = np.arange(begin_time, end_time+0.01, simDataDict["Tstep"]);
    ax3.plot(time, simDataDict["pr_status"]);

# Acceleration
fig4, ax4 = plt.subplots();
id_cav = 1;
time = np.arange(begin_time, end_time+0.01, simDataDict["Tstep"]);
ax4.plot(time, simDataDict["S"][int(begin_time/simDataDict["Tstep"]-1):int(end_time/simDataDict["Tstep"]), 0, 2], color='black', linewidth=(line_width-0.5));
for i in range(0, n_vehicle):
    if simDataDict["ID"][0, i] == 1:
        if id_cav == 1:
            ax4.plot(time, simDataDict["S"][int(begin_time/simDataDict["Tstep"]-1):int(end_time/simDataDict["Tstep"]), i+1, 2], color='red', linewidth=line_width); # line for velocity of CAVs
            id_cav  = id_cav+1;
        else: #if id_cav == 2
            ax4.plot(time, simDataDict["S"][int(begin_time/simDataDict["Tstep"]-1):int(end_time/simDataDict["Tstep"]), i+1, 2],color='blue', linewidth=line_width); # line for velocity of CAVs
ax4.grid(True);
fig4.gca().tick_params(labelsize=total_size);
fig4.gca().ticklabel_format(useMathText=True);
ax4.set_ylim(-6, 4);
ax4.set_xlim(0, 30);

ax4.set_xlabel('$t$ [$\mathrm{s}$]', fontsize=label_size, color='black');
ax4.set_ylabel('Acceleration [$\mathrm{m/s^2}$]', fontsize=label_size, color='black');

# if mix
#     print(gcf,['.\figs\BrakePerturbation_AccelerationProfile_Controller_',num2str(controller_type)],'-painters','-depsc2','-r300');
# else
#     print(gcf,'.\figs\BrakePerturbation_AccelerationProfile_AllHDVs','-painters','-depsc2','-r300');
# end

# -------------------------------------------------------------------------
#   Calculate Performance Indexes
#--------------------------------------------------------------------------

smooth_window = 9;
for i in range(1, n_vehicle+1):
    simDataDict["S"][:, i, 2] = smooth(simDataDict["S"][:, i, 2], smooth_window);
   #simDataDict["S"][:, i, 2] = uniform_filter1d(simDataDict["S"][:, i, 2], smooth_window, axis=0)
   #simDataDict["S"][:,i,2]   = np.convolve(simDataDict["S"][:,i,2], np.ones(smooth_window)/smooth_window, mode='valid');
   #smooth(S[:,i,2],smooth_window); ??

FuelConsumption = 0;
VelocityError   = 0;
for i in range(int(begin_time/simDataDict["Tstep"]-1), int(end_time/simDataDict["Tstep"])):
    R  = 0.333 + np.dot(0.00108, simDataDict["S"][i,3:,1]**2) + np.dot(1.2, simDataDict["S"][i,3:,2]);
    R = np.around(R, decimals=4);
    Fuel  = 0.444 + np.dot(0.09,R)*simDataDict["S"][i,3,1] + (np.dot(0.054, np.maximum(0,simDataDict["S"][i,3:,2])**2))*simDataDict["S"][i,3:,1];
    Fuel[R <= 0] = 0.444;
    Fuel = np.around(Fuel, decimals=4);
    FuelConsumption = FuelConsumption + np.dot(np.sum(Fuel), simDataDict["Tstep"]);
    
    VelocityError = VelocityError + np.sum(np.abs(simDataDict["S"][i,3:,1]-simDataDict["S"][i,0,1])/simDataDict["S"][i,0,1]);

VelocityError = VelocityError/n_vehicle/((end_time-begin_time)/simDataDict["Tstep"]);

print("Fuel comsumption:   %4.2f \n" % (FuelConsumption));
print("Velocity error:   %4.2f \n" % (VelocityError));

plt.show();
