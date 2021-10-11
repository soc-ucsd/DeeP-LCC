% =========================================================================
%               Analysis for simulation results under same data sample               
% =========================================================================

clc; clear; close all;

% Data set
data_str        = '2';  % 1. random ovm  2. manual ovm  3. homogeneous ovm
% Mix or not
mix             = 1;    % 0. all HDVs; 1. mix
% Type of the controller
controller_type = 2;    % 1. DeePC  2. MPC  3.SPC  4. SPC without Regulation
% Type for HDV car-following model
hdv_type        = 1;    % 1. OVM   2. IDM
% Uncertainty for HDV behavior
acel_noise      = 0.1;  % A white noise signal on HDV's original acceleration
% Perturbation amplitude
per_type         = 1; % 1. sinuoid perturbation 2. brake perturbation 3. ngsim simulation
ngsim_id         = '2131';
per_amp          = 5;
% Whether there exists constraints
constraint_bool  = 1;

end_time         = 40;              % Total Simulation Time
begin_time       = 20;

weight_v     = 1;        % weight coefficient for velocity error
weight_s     = 0.5;      % weight coefficient for spacing error   
weight_u     = 0.1;      % weight coefficient for control input

lambda_g     = 100;      % penalty on ||g||_2^2 in objective
lambda_y     = 1e4;      % penalty on ||sigma_y||_2^2 in objective

i_data          = 1;

if mix
switch controller_type
    case 1
        if per_type ~= 3
            if constraint_bool
            load(['_data\simulation_data\DeePC\constrained_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
                '_hdvType_',num2str(hdv_type),'_lambdaG_',num2str(lambda_g),'_lambdaY_',num2str(lambda_y),'.mat']);
            controller_str = 'DeePC';
        else
            load(['_data\simulation_data\DeePC\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
                '_hdvType_',num2str(hdv_type),'_lambdaG_',num2str(lambda_g),'_lambdaY_',num2str(lambda_y),'.mat']);
            controller_str = 'DeePC';
        end
        else % ngsim simulation
            load(['_data\simulation_data\DeePC\ngsim_simulation\simulation_data',data_str,'_',num2str(i_data),'_ngsim_',ngsim_id,'_noiseLevel_',num2str(acel_noise),...
                '_hdvType_',num2str(hdv_type),'_lambdaG_',num2str(lambda_g),'_lambdaY_',num2str(lambda_y),'.mat']);
            controller_str = 'DeePC';
        end
    case 2
        if constraint_bool
        load(['_data\simulation_data\MPC\constrained_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),'_hdvType_1.mat']);
        controller_str = 'MPC';
        else
        load(['_data\simulation_data\MPC\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),'_hdvType_1.mat']);
        controller_str = 'MPC';    
        end
    case 3
%         load(['_data\simulation_data\SPC\simulation_data2_',num2str(i_data),'_noiseLevel_',num2str(acel_noise),...
%             '_hdvType_',num2str(hdv_type),'_lambdaY_',num2str(lambda_y),'.mat']);
        load(['_data\simulation_data\SPC\simulation_data',data_str,'_',num2str(i_data),'_perturbationAmp_',num2str(per_amp),'_noiseLevel_',num2str(acel_noise),...
            '_hdvType_',num2str(hdv_type),'_lambdaY_',num2str(lambda_y),'.mat']);
        controller_str = 'SPC';
    case 4
        load(['_data\simulation_data\SPC_withoutRegulation\simulation_data',data_str,'_',num2str(i_data),'_perturbationAmp_',num2str(per_amp),'_noiseLevel_',num2str(acel_noise),...
            '_hdvType_',num2str(hdv_type),'_lambdaY_',num2str(lambda_y),'.mat']);
        controller_str = 'SPC without Regulation';
end
else
    if per_type ~= 3
        if constraint_bool
            load(['_data\simulation_data\HDV\constrained_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
                '_hdvType_',num2str(hdv_type),'_lambdaG_',num2str(lambda_g),'_lambdaY_',num2str(lambda_y),'.mat']);
            controller_str = 'DeePC';
        else
            load(['_data\simulation_data\HDV\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
                '_hdvType_',num2str(hdv_type),'_lambdaG_',num2str(lambda_g),'_lambdaY_',num2str(lambda_y),'.mat']);
            controller_str = 'DeePC';
        end
    else % ngsim simulation
        load(['_data\simulation_data\HDV\ngsim_simulation\simulation_data',data_str,'_',num2str(i_data),'_ngsim_',ngsim_id,'_noiseLevel_',num2str(acel_noise),...
            '_hdvType_',num2str(hdv_type),'.mat']);
        controller_str = 'HDV';
    end
end


n_vehicle   = length(ID);           % number of vehicles


% -------------------------------------------------------------------------
%   Plot Results
%--------------------------------------------------------------------------

figure;
plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,1,2),'Color',[0.5,0.5,0.5]); hold on;
for i = 1:n_vehicle
   if ID(i) == 1
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color','r'); hold on; % line for velocity of CAVs
    else
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color','b'); hold on; % line for velocity of HDVs
    end 
end
grid on;
title(controller_str,'Interpreter','latex');
set(gca,'TickLabelInterpreter','latex','fontsize',14);

set(gcf,'Position',[250 150 500 350]);
fig = gcf;
fig.PaperPositionMode = 'auto';

figure;

for i = 1:n_vehicle
   if ID(i) == 1
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i,1)-S(begin_time/Tstep:end_time/Tstep,i+1,1),'Color','r'); hold on; % line for velocity of CAVs
    else
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i,1)-S(begin_time/Tstep:end_time/Tstep,i+1,1),'Color','b'); hold on; % line for velocity of HDVs
    end 
end
grid on;
title(controller_str,'Interpreter','latex');
set(gca,'TickLabelInterpreter','latex','fontsize',14);

set(gcf,'Position',[250 150 500 350]);
fig = gcf;
fig.PaperPositionMode = 'auto';
