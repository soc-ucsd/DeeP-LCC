% =========================================================================
%               Analysis for simulation results under multiple data samples
%               Calculate the average quadratic cost
% =========================================================================

clc; clear; close all;

% Type for HDV car-following model
hdv_type        = 1;    % 1. OVM   2. IDM
% Uncertainty for HDV behavior
acel_noise      = 0.1;  % A white noise signal on HDV's original acceleration
% Data number
data_number     = 100;
% Perturbation amplitude
per_type         = 1; % 1. sinuoid perturbation 2. brake perturbation 
per_amp          = 5;
% Whether there exists constraints
constraint_bool  = 0;

weight_v     = 1;        % weight coefficient for velocity error
weight_s     = 0.5;      % weight coefficient for spacing error   
weight_u     = 0.1;      % weight coefficient for control input

lambda_g     = 100;      % penalty on ||g||_2^2 in objective
lambda_y     = 1e4;      % penalty on ||sigma_y||_2^2 in objective

% Data set
data_str        = '3';  % 1. random ovm  2. manual ovm  3. homogeneous ovm

switch hdv_type
    case 1
        % Driver Model: OVM
        load(['_data/hdv_ovm_',data_str,'.mat']);
    case 2
        % Driver Model: IDM
        load('_data/hdv_idm.mat');
%         v_max   = 30;
%         T_gap   = 1;
%         a       = 1;
%         b       = 1.5;
%         delta   = 4;
%         s_st    = 5;
%         % Equilibrium spacing
%         s_star  = (s_st+T_gap*v_star)/sqrt(1-(v_star/v_max)^delta);        
end

% Equilibrium Spacing
s_star = hdv_parameter.s_star;


% -------------------------------------------------------------------------
%   Calculate statistics
%--------------------------------------------------------------------------
lq_cost             = zeros(data_number,4);
velocity_error      = zeros(data_number,4);
spacing_error       = zeros(data_number,4);
 

% Type of the controller
for controller_type = 1:2    % 1. DeePC  2. MPC  3.SPC  4. SPC without Regulation

for i_data          = 1:data_number

switch controller_type
    case 1
        load(['_data\simulation_data\DeePC\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
                '_hdvType_',num2str(hdv_type),'_lambdaG_',num2str(lambda_g),'_lambdaY_',num2str(lambda_y),'.mat']);
    case 2
        load(['_data\simulation_data\MPC\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),'_hdvType_1.mat']);
    case 3
%         load(['_data\simulation_data\SPC\simulation_data2_',num2str(i_data),'_noiseLevel_',num2str(acel_noise),...
%             '_hdvType_',num2str(hdv_type),'_lambdaY_',num2str(lambda_y),'.mat']);
        load(['_data\simulation_data\SPC\simulation_data',num2str(data_str),'_',num2str(i_data),'_perturbationAmp_',num2str(per_amp),'_noiseLevel_',num2str(acel_noise),...
            '_hdvType_',num2str(hdv_type),'_lambdaY_',num2str(lambda_y),'.mat']);
    case 4
        load(['_data\simulation_data\SPC_withoutRegulation\simulation_data',num2str(data_str),'_',num2str(i_data),'_perturbationAmp_',num2str(per_amp),'_noiseLevel_',num2str(acel_noise),...
            '_hdvType_',num2str(hdv_type),'_lambdaY_',num2str(lambda_y),'.mat']);
end

n_vehicle   = length(ID);           % number of vehicles

for i = 1:n_vehicle
    if ID(i) == 1
        lq_cost(i_data,controller_type) = lq_cost(i_data,controller_type) + ...
                                          weight_v*sum((S(:,i+1,2)-v_star).^2) + ...
                                          weight_s*sum((S(:,i,1)-S(:,i+1,1)-s_star(i)).^2) + ...
                                          weight_u*sum(S(:,i+1,3).^2);
    else
        lq_cost(i_data,controller_type) = lq_cost(i_data,controller_type) + ...
                                          weight_v*sum((S(:,i+1,2)-v_star).^2);
    end
    velocity_error(i_data,controller_type) = velocity_error(i_data,controller_type) + ...
                                             sum((S(:,i+1,2)-v_star).^2);
    spacing_error(i_data,controller_type) = spacing_error(i_data,controller_type) + ...
                                             sum((S(:,i,1)-S(:,i+1,1)-s_star(i)).^2);
end

end

end
% -------------------------------------------------------------------------
%   Plot Results
% -------------------------------------------------------------------------

%-----------------
% Real Cost
%-----------------
figure;
for controller_type = 1:2    % 1. DeePC  2. MPC  3.SPC
    plot(lq_cost(:,controller_type)); hold on;
end
grid on;
% l = legend('DeePC','MPC','SPC','SPC without Regulation');
l = legend('DeePC','MPC');
l.Interpreter = 'latex';
l.Box = 'off';

set(gca,'TickLabelInterpreter','latex','fontsize',14);
title('Real Cost','Interpreter','latex');

set(gcf,'Position',[250 150 500 350]);
fig = gcf;
fig.PaperPositionMode = 'auto';


%-----------------
% Velocity Error
%-----------------
figure;
for controller_type = 1:2    % 1. DeePC  2. MPC  3.SPC
    plot(velocity_error(:,controller_type)); hold on;
end
grid on;
% l = legend('DeePC','MPC','SPC','SPC without Regulation');
l = legend('DeePC','MPC');
l.Interpreter = 'latex';
l.Box = 'off';

set(gca,'TickLabelInterpreter','latex','fontsize',14);
title('Aggregrated Velocity Error','Interpreter','latex');

set(gcf,'Position',[250 150 500 350]);
fig = gcf;
fig.PaperPositionMode = 'auto';


%-----------------
% Spacing Error
%-----------------
figure;
for controller_type = 1:2    % 1. DeePC  2. MPC  3.SPC
    plot(spacing_error(:,controller_type)); hold on;
end
grid on;
% l = legend('DeePC','MPC','SPC','SPC without Regulation');
l = legend('DeePC','MPC');
l.Interpreter = 'latex';
l.Box = 'off';

set(gca,'TickLabelInterpreter','latex','fontsize',14);
title('Aggregrated Spacing Error','Interpreter','latex');

set(gcf,'Position',[250 150 500 350]);
fig = gcf;
fig.PaperPositionMode = 'auto';


