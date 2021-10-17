% =========================================================================
%               Analysis for simulation results under same data sample
% =========================================================================

clc; clear; close all;

addpath('arrow.m');

% Data set
data_str        = '2';  % 1. random ovm  2. manual ovm  3. homogeneous ovm
% Mix or not
mix             = 1;    % 0. all HDVs; 1. mix
% Type of the controller
controller_type = 2;    % 1. DeeP-LCC  2. MPC  
% Type for HDV car-following model
hdv_type        = 1;    % 1. OVM   2. IDM
% Uncertainty for HDV behavior
acel_noise      = 0.1;  % A white noise signal on HDV's original acceleration

% Head vehicle trajectory
trajectory_id = '1';
head_vehicle_trajectory = load(['../_data/nedc_modified_v',num2str(trajectory_id),'.mat']);
end_time = head_vehicle_trajectory.time(end);

initialization_time = 30;               % Time for the original HDV-all system to stabilize
adaption_time       = 20;               % Time for the CAVs to adjust to their desired state
total_time          = initialization_time + adaption_time + end_time;  % Total Simulation Time
begin_time          = initialization_time + adaption_time;

weight_v     = 1;        % weight coefficient for velocity error
weight_s     = 0.5;      % weight coefficient for spacing error
weight_u     = 0.1;      % weight coefficient for control input

lambda_g     = 100;      % penalty on ||g||_2^2 in objective
lambda_y     = 1e4;      % penalty on ||sigma_y||_2^2 in objective

i_data          = 1;

if mix
    switch controller_type
        case 1
            load(['..\_data\simulation_data\DeeP_LCC\nedc_simulation\simulation_data',data_str,'_',num2str(i_data),'_modified_v',num2str(trajectory_id),'_noiseLevel_',num2str(acel_noise),...
                '_hdvType_',num2str(hdv_type),'_lambdaG_',num2str(lambda_g),'_lambdaY_',num2str(lambda_y),'.mat']);
            controller_str = 'DeePC';
        case 2
            load(['..\_data\simulation_data\MPC\nedc_simulation\simulation_data',data_str,'_modified_v',num2str(trajectory_id),'_noiseLevel_',num2str(acel_noise),...
                '_hdvType_',num2str(hdv_type),'.mat']);
            controller_str = 'MPC';
    end
else % ngsim simulation
    load(['..\_data\simulation_data\HDV\nedc_simulation\simulation_data',data_str,'_modified_v',num2str(trajectory_id),'_noiseLevel_',num2str(acel_noise),...
        '_hdvType_',num2str(hdv_type),'.mat']);
    controller_str = 'HDV';
    
end


n_vehicle   = length(ID);           % number of vehicles


% -------------------------------------------------------------------------
%   Plot Results
%--------------------------------------------------------------------------
color_gray  = [170 170 170]/255;
color_red   = [244, 53, 124]/255;
color_blue  = [67, 121, 227]/255;
color_black = [0 0 0];
color_orange = [255,132,31]/255;
color_blue_2 = [61, 90, 128]/255;
color_red_2  = [238, 108, 77]/255;
label_size  = 18;
total_size  = 14;
line_width  = 1.5;

% Head vehicle trajectory
time_point = [60,88,121,176,206];  % For Trajectory V1
% time_point = [60,88,121,166,196];   % For Trajectory V2
time_scale = (begin_time:Tstep:total_time)-(initialization_time + adaption_time);

figure;
plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),1,2),'Color',color_black,'linewidth',line_width); hold on;
grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'XLim',[begin_time total_time]-(initialization_time + adaption_time));
set(gca,'YLim',[10,30]);


for time_i = 1:4
    plot(time_point(time_i)*ones(20,1)-(initialization_time + adaption_time),linspace(10,S(time_point(time_i)/Tstep,1,2),20),'--','Color',color_blue,'linewidth',line_width); hold on;
    text_phase = text((time_point(time_i)+time_point(time_i+1))/2-(initialization_time + adaption_time),11.5,['phase ',num2str(time_i)],...
        'Interpreter','latex','FontSize',label_size,'HorizontalAlignment','center','Color',color_red);
end




xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Velocity [$\mathrm{m/s}$]','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[250 550 850 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';

% print(gcf,'.\figs\NEDC_HeadVehicleTrajectory','-painters','-depsc2','-r300');


% Velocity
% for time_i = 1:4
%     begin_time = time_point(time_i);
%     total_time = time_point(time_i+1);
%     figure;
%     plot(begin_time:Tstep:total_time,S(begin_time/Tstep:round(total_time/Tstep),1,2),'Color',color_gray,'linewidth',line_width); hold on;
%     for i = 1:n_vehicle
%         if ID(i) == 1
%             plot(begin_time:Tstep:total_time,S(begin_time/Tstep:round(total_time/Tstep),i+1,2),'Color',color_red,'linewidth',line_width); hold on; % line for velocity of CAVs
%         else
%             plot(begin_time:Tstep:total_time,S(begin_time/Tstep:round(total_time/Tstep),i+1,2),'Color',color_blue,'linewidth',line_width); hold on; % line for velocity of HDVs
%         end
%     end
%     grid on;
%     set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
%     set(gca,'XLim',[begin_time total_time]);
%     
%     xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
%     yl = ylabel('Velocity [$\mathrm{m/s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
%     
%     set(gcf,'Position',[250 150 400 300]);
%     fig = gcf;
%     fig.PaperPositionMode = 'auto';
%     
% end

total_time          = initialization_time + adaption_time + end_time;  % Total Simulation Time
begin_time          = initialization_time + adaption_time;

figure;
% h = axes('position',[0 0 1 1]);
% axis(h);
plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),1,2),'Color',color_black,'linewidth',line_width); hold on;
for i = 1:n_vehicle
    if ID(i) == 0
        plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),i+1,2),'Color',color_gray,'linewidth',line_width/2); hold on; % line for velocity of HDVs
    end
end
id_cav = 1;
for i = 1:n_vehicle
    if ID(i) == 1
        if id_cav == 1
            plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),i+1,2),'Color',color_red,'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav = id_cav+1;
        elseif id_cav == 2
            plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),i+1,2),'Color',color_blue,'linewidth',line_width); hold on; % line for velocity of CAVs
        end
    end
end

for time_i = 1:4
    plot(time_point(time_i)*ones(20,1)-(initialization_time + adaption_time),linspace(10,S(time_point(time_i)/Tstep,1,2),20),'--','Color',color_blue_2,'linewidth',line_width/2); hold on;
    text_phase = text((time_point(time_i)+time_point(time_i+1))/2-(initialization_time + adaption_time),12.5,['Phase ',num2str(time_i)],...
        'Interpreter','latex','FontSize',label_size,'HorizontalAlignment','center','Color',color_red_2);
end

grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'XLim',[time_scale(1) time_scale(end)]);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Velocity [$\mathrm{m/s}$]','fontsize',label_size,'Interpreter','latex','Color','k');

set(gca,'YLim',[11.5,28.5]);

set(gcf,'Position',[250 150 850 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';

% h1 = axes;
% axis(h1);
% begin_time_1 = 65;
% total_time_1 = 75;
% 
% plot(begin_time_1:Tstep:total_time_1,S(begin_time_1/Tstep:round(total_time_1/Tstep),1,2),'Color',color_black,'linewidth',line_width); hold on; % line for velocity of HDVs
% for i = 1:n_vehicle
%     if ID(i) == 0
%         plot(begin_time_1:Tstep:total_time_1,S(begin_time_1/Tstep:round(total_time_1/Tstep),i+1,2),'Color',color_gray,'linewidth',line_width/2); hold on; % line for velocity of HDVs
%     end
% end
% id_cav = 1;
% for i = 1:n_vehicle
%     if ID(i) == 1
%         if id_cav == 1
%             plot(begin_time_1:Tstep:total_time_1,S(begin_time_1/Tstep:round(total_time_1/Tstep),i+1,2),'Color',color_red,'linewidth',line_width); hold on; % line for velocity of CAVs
%             id_cav = id_cav+1;
%         elseif id_cav == 2
%             plot(begin_time_1:Tstep:total_time_1,S(begin_time_1/Tstep:round(total_time_1/Tstep),i+1,2),'Color',color_blue,'linewidth',line_width); hold on; % line for velocity of CAVs
%         end
%     end
% end
% 
% set(gca,'xticklabel',[])
% set(gca,'yticklabel',[])
% set(gca,'YLim',[13 18]);
% h1.Position = [0.15 0.6 0.15 0.25];

% if mix
%     print(gcf,['.\figs\NEDC_VelocityProfile_Controller_',num2str(controller_type)],'-painters','-depsc2','-r300');
% else
%     print(gcf,'.\figs\NEDC_VelocityProfile_AllHDVs','-painters','-depsc2','-r300');
% end





% Spacing
figure;
for i = 1:n_vehicle
    if ID(i) == 0
        plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),i,1)-S(begin_time/Tstep:round(total_time/Tstep),i+1,1),'Color',color_gray,'linewidth',line_width/2); hold on; % line for velocity of HDVs
    end
end
id_cav = 1;
for i = 1:n_vehicle
    if ID(i) == 1
        if id_cav == 1
            plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),i,1)-S(begin_time/Tstep:round(total_time/Tstep),i+1,1),'Color',color_red,'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav = id_cav+1;
        elseif id_cav == 2
            plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),i,1)-S(begin_time/Tstep:round(total_time/Tstep),i+1,1),'Color',color_blue,'linewidth',line_width); hold on; % line for velocity of CAVs
        end
    end
end

grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'XLim',[time_scale(1) time_scale(end)]);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Spacing [$\mathrm{m}$]','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[1050 150 750 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';

% if mix
%     print(gcf,['.\figs\NGSIM_SpacingProfile_Controller_',num2str(controller_type)],'-painters','-depsc2','-r300');
% else
%     print(gcf,'.\figs\NGSIM_SpacingProfile_AllHDVs','-painters','-depsc2','-r300');
% end

% acceleration
figure;


for i = 1:n_vehicle
    % smooth acceleration signal
    S(:,i+1,3) = smooth(S(:,i+1,3),10);
end
for i = 1:n_vehicle
    if ID(i) == 0
        plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),i+1,3),'Color',color_gray,'linewidth',line_width/2); hold on; % line for velocity of HDVs
    end
end
id_cav = 1;
for i = 1:n_vehicle
    if ID(i) == 1
        if id_cav == 1
            plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),i+1,3),'Color',color_red,'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav = id_cav+1;
        elseif id_cav == 2
            plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),i+1,3),'Color',color_blue,'linewidth',line_width); hold on; % line for velocity of CAVs
        end
    end
end
grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'XLim',[time_scale(1) time_scale(end)]);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Acceleration [$\mathrm{m/s^2}$]','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[1050 550 700 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';

% -------------------------------------------------------------------------
%   Calculate Performance Indexes
%--------------------------------------------------------------------------
FuelConsumption = 0;
VelocityError   = 0;
for i=begin_time/Tstep:total_time/Tstep
    R  = 0.333 + 0.00108*S(i,4:end,2).^2 + 1.2*S(i,4:end,3);
    Fuel  = 0.444 + 0.09*R.*S(i,4:end,2) + 0.054 * max(0,S(i,4:end,3)).^2.*S(i,4:end,2);
    Fuel(R <= 0) = 0.444;
    FuelConsumption = FuelConsumption + sum(Fuel)*Tstep;
    
    VelocityError = VelocityError + sum(abs(S(i,4:end,2)-S(i,1,2))/S(i,1,2));
    
end

VelocityError = VelocityError/n_vehicle/((total_time-begin_time)/Tstep);

fprintf('Fuel comsumption:   %4.2f \n',FuelConsumption);
fprintf('Velocity error:   %4.4f \n',VelocityError);
