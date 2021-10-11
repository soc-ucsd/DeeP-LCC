% =========================================================================
%               Analysis for simulation results under same data sample               
% =========================================================================

clc; clear; close all;

% Data set
data_str         = '3';  % 1. random ovm  2. manual ovm  3. homogeneous ovm
% Mix or not
mix              = 1;    % 0. all HDVs; 1. mix
% Type of the controller
controller_type  = 1;    % 1. DeePC  2. MPC  3.SPC  4. SPC without Regulation
% Type for HDV car-following model
hdv_type         = 1;    % 1. OVM   2. IDM
% Uncertainty for HDV behavior
acel_noise       = 0.1;  % A white noise signal on HDV's original acceleration
% Perturbation amplitude
per_type         = 1;   % 1. sinuoid perturbation 2. brake perturbation 3. ngsim simulation
per_amp          = 5;
% Whether there exists constraints
constraint_bool  = 0;

% Simulation Time
begin_time       = 10;
end_time         = 40;              

data_number      = 100;

weight_v     = 1;        % weight coefficient for velocity error
weight_s     = 0.5;      % weight coefficient for spacing error   
weight_u     = 0.1;      % weight coefficient for control input

lambda_g     = 100;      % penalty on ||g||_2^2 in objective
lambda_y     = 1e4;      % penalty on ||sigma_y||_2^2 in objective

switch hdv_type
    case 1
        % Driver Model: OVM
        load(['_data/hdv_ovm_',data_str,'.mat']);
    case 2
        % Driver Model: IDM
        load('_data/hdv_idm.mat');      
end

% Equilibrium Spacing
s_star = hdv_parameter.s_star;


% -------------------------------------------------------------------------
%   Calculate statistics
%--------------------------------------------------------------------------
real_cost             = zeros(data_number,2);
 

% Type of the controller
for controller_type = 1:2    % 1. DeePC  2. MPC 

for i_data          = 1:data_number

switch controller_type
    case 1
        load(['_data\simulation_data\DeePC\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
                '_hdvType_',num2str(hdv_type),'_lambdaG_',num2str(lambda_g),'_lambdaY_',num2str(lambda_y),'.mat']);
    case 2
        load(['_data\simulation_data\MPC\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),'_hdvType_1.mat']);
end

n_vehicle   = length(ID);           % number of vehicles

for i = 1:n_vehicle
    if ID(i) == 1
        real_cost(i_data,controller_type) = real_cost(i_data,controller_type) + ...
                                          weight_v*sum((S(:,i+1,2)-v_star).^2) + ...
                                          weight_s*sum((S(:,i,1)-S(:,i+1,1)-s_star(i)).^2) + ...
                                          weight_u*sum(S(:,i+1,3).^2);
    else
        real_cost(i_data,controller_type) = real_cost(i_data,controller_type) + ...
                                          weight_v*sum((S(:,i+1,2)-v_star).^2);
    end
end

end

end

average_cost = mean(real_cost);
% -------------------------------------------------------------------------
%   Plot Results
% -------------------------------------------------------------------------

color_gray  = [190 190 190]/255;
color_red   = [244, 53, 124]/255;
color_blue  = [67, 121, 227]/255;
color_black = [0 0 0];
color_blue_2 = [61, 90, 128]/255;
color_red_2  = [238, 108, 77]/255;
label_size  = 18;
total_size  = 16;
line_width  = 1;
%-----------------
% Real Cost
%-----------------
figure;
% controller_type = 1:2    % 1. DeePC  2. MPC  3.SPC
p1 = plot(real_cost(:,1),'Color',color_blue_2,'Linewidth',line_width); hold on;
plot(average_cost(1)*ones(1,data_number),'--','Color',color_blue_2,'Linewidth',line_width); hold on;
p2 = plot(real_cost(:,2),'Color',color_red_2,'Linewidth',line_width); hold on;
plot(average_cost(2)*ones(1,data_number),'--','Color',color_red_2,'Linewidth',line_width); hold on;


grid on;
l = legend([p1 p2],'DeePC','MPC');
l.Interpreter = 'latex';
l.FontSize = label_size;
l.Box = 'off';

set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'YLim',[5.5e4,6.1e4]);

grid on;

xl = xlabel('Pre-collected Trajectory No.','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Real Cost','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[250 150 750 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';

print(gcf,'.\figs\SinusoidPerturbation_RealCost','-painters','-depsc2','-r300');

fprintf('Average Cost:   DeePC  |    MPC    \n');
fprintf('             %4.2f  |  %4.2f  \n',average_cost);