% =========================================================================
%       Traffic Simulation with Already Collected Data
%       Scenario: general case with multiple CAVs and multiple HDVs
%       The head vehicle has a sine-wave perturbation
% =========================================================================

clc; close all; clear;
addpath('_fcn');

% -------------------------------------------------------------------------
%   Parameter setup
% -------------------------------------------------------------------------

% Data number
data_number     = 1;
% Data set
data_str        = '2';  % 1. random ovm  2. manual ovm  3. homogeneous ovm
% Whether there exist constraints
constraint_bool  = 1;

% Type of the controller
controller_type = 1;    % 1. DeePC  2. MPC  3.SPC  4. SPC without Regulation

% Type for HDV car-following model
hdv_type        = 1;    % 1. OVM   2. IDM
% Uncertainty for HDV behavior
acel_noise      = 0.1;    % A white noise signal on HDV's original acceleration

% Head vehicle trajectory
ngsim_id_collected  = [2131,2067,1351,1336,1648,1469];
end_time_collected  = [62.6,56.9,49.1,50.0,65.9,64.8];

for ngsim_i = 1:length(ngsim_id_collected)
ngsim_id        = ngsim_id_collected(ngsim_i);
end_time        = end_time_collected(ngsim_i);

% NGSIM data
head_vehicle_trajectory = load(['./_data/ngsim_data/',num2str(ngsim_id),'.mat']);

% Parameters in Simulation
initialization_time = 30;
total_time          = initialization_time + end_time;  % Total Simulation Time
Tstep               = 0.05;            % Time Step
total_time_step     = round(total_time/Tstep);


% DeePC Formulation
T       = 2000;      % length of data samples
Tini    = 20;        % length of past data
N       = 50;        % length of predicted horizon

weight_v     = 1;        % weight coefficient for velocity error
weight_s     = 0.5;      % weight coefficient for spacing error   
weight_u     = 0.1;      % weight coefficient for control input

lambda_g     = 100;      % penalty on ||g||_2^2 in objective
lambda_y     = 1e4;      % penalty on ||sigma_y||_2^2 in objective

% for lambda_y = [1,10,1e2,1e3,1e4,1e5,1e6]
    
% System Dynamics
% vel_noise = 0.1;         % noise signal in velocity signal

% ------------------------------------------
% Parameters in Mixed Traffic
% ------------------------------------------
ID          = [0,0,1,0,0,1,0,0];    % ID of vehicle types
                                    % 1: CAV  0: HDV
pos_cav     = find(ID==1);          % position of CAVs
n_vehicle   = length(ID);           % number of vehicles
n_cav       = length(pos_cav);      % number of CAVs
n_hdv       = n_vehicle-n_cav;      % number of HDVs

mix         = 0;                    % whether mixed traffic flow

v_star      = 15;                   % Equilibrium velocity
s_star      = 20;                   % Equilibrium spacing for CAV

% Constraints
acel_max        = 2;
dcel_max        = -5;
spacing_max     = 40;
spacing_min     = 5;
u_limit = [dcel_max,acel_max];
s_limit = [spacing_min,spacing_max]-s_star;

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


    
% What is measurable
% for measure_type    = 2:3
measure_type = 3;
% 1. Only the velocity errors of all the vehicles are measurable;
% 2. All the states, including velocity error and spacing error are measurable;
% 3. Velocity error and spacing error of the CAVs are measurable, 
%    and the velocity error of the HDVs are measurable.


% ------------------
%  size in DeePC
% ------------------

n_ctr = 2*n_vehicle;    % number of state variables
m_ctr = n_cav;          % number of input variables
switch measure_type     % number of output variables
    case 1
        p_ctr = n_vehicle;
    case 2
        p_ctr = 2*n_vehicle;
    case 3
        p_ctr = n_vehicle + n_cav;
end

for i_data = 1:data_number
    
load(['_data\trajectory_data_collection\data',data_str,'_',num2str(i_data),'_noiseLevel_',num2str(acel_noise),'.mat']);


% -------------------------------------------------------------------------
%   Scenario initialization
%-------------------------------------------------------------------------- 

% There is one head vehicle at the very beginning
S           = zeros(total_time_step,n_vehicle+1,3);
S(1,1,1)    = 0;
for i = 2 : n_vehicle+1
    S(1,i,1) = S(1,i-1,1) - hdv_parameter.s_star(i-1);
end
S(1,:,2)    = v_star * ones(n_vehicle+1,1);


% ------------------
%  DeePC Formulation
% ------------------
Q_v         = weight_v*eye(n_vehicle);         % penalty for velocity error
Q_s         = weight_s*eye(p_ctr-n_vehicle);   % penalty for spacing error
Q           = blkdiag(Q_v,Q_s);                % penalty for trajectory error
R           = weight_u*eye(m_ctr);             % penalty for control input

u           = zeros(m_ctr,total_time_step); % control input
x           = zeros(n_ctr,total_time_step); % state variables
y           = zeros(p_ctr,total_time_step); % output variables
pr_status   = zeros(total_time_step,1);     % problem status
e           = zeros(1,total_time_step);     % external input

% ------------------
%  Reference trajectory
% ------------------
r       = zeros(p_ctr,total_time_step+N);            % stabilization

% -------------------------------------------------------------------------
%   Simulation
%--------------------------------------------------------------------------
tic
% ------------------
%  Initialization
% ------------------
for k = 1:initialization_time/Tstep-1
    % Update acceleration
    acel         =  HDV_dynamics(S(k,:,:),hdv_parameter) ...
                    -acel_noise + 2*acel_noise*rand(n_vehicle,1);
    
    S(k,1,3)           = 0;               % the head vehicle
    S(k,2:end,3)       = acel;            % all the vehicles using HDV model
    
    S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
    S(k+1,1,2) = head_vehicle_trajectory.vel(1);    % the velocity of the head vehicle
    S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
    
    v_star = head_vehicle_trajectory.vel(1);
    y(:,k) = measure_mixed_traffic(S(k,2:end,2),S(k,:,1),ID,v_star,s_star,measure_type);
    e(k)   = S(k,1,2) - v_star;
    u(:,k) = S(k,pos_cav+1,3);
        
end

% update past data in control process
uini = u(:,k-Tini+1:k);
yini = y(:,k-Tini+1:k);
eini = S(k-Tini+1:k,1,2) - v_star;


% ------------------
%  simulation starts here
% ------------------
for k = initialization_time/Tstep:total_time_step-1
    % Update acceleration
    acel         =  HDV_dynamics(S(k,:,:),hdv_parameter) ...
                    -acel_noise + 2*acel_noise*rand(n_vehicle,1);
    
    S(k,2:end,3) = acel;     % all the vehicles using HDV model
    
    if mix
        switch controller_type
            case 1
                % Calculate control input via DeePC
                if constraint_bool
                    [u_opt,y_opt,pr] = qpDeePC_withExternalInput(Up,Yp,Uf,Yf,Ep,Ef,uini,yini,eini,Q,R,r(:,k:k+N-1),...
                        lambda_g,lambda_y,u_limit,s_limit);
                else
                    [u_opt,y_opt,pr] = qpDeePC_withExternalInput(Up,Yp,Uf,Yf,Ep,Ef,uini,yini,eini,Q,R,r(:,k:k+N-1),...
                        lambda_g,lambda_y);
                end
            case 2
                % Calculate control input via MPC
                [u_opt,y_opt,pr] = MPC(ID,Tstep,hdv_type,measure_type,v_star,uini,yini,N,Q,R,r(:,k:k+N-1));
            case 3
                % Calaulate control input via SPC(ARX)
                [u_opt,y_opt,pr] = qpSPC_withExternalInput(Up,Yp,Uf,Yf,Ep,Ef,uini,yini,eini,Q,R,r(:,k:k+N-1),lambda_y);
            case 4
                [u_opt,y_opt,pr] = qpSPC_withExternalInput_withoutRegulation(Up,Yp,Uf,Yf,Ep,Ef,uini,yini,eini,Q,R,r(:,k:k+N-1),lambda_y);
        end
        % One-step formulation
        u(:,k) = u_opt(1:m_ctr,1);
        % Update accleration for the CAV
        S(k,pos_cav+1,3)   = u(:,k);
        % Judge whether SD system commands to brake
        brake_vehicle_ID = find(acel==dcel_max);                % the vehicles that need to brake
        brake_cav_ID     = intersect(brake_vehicle_ID,pos_cav); % the CAVs that need to brake
        if ~isempty(brake_cav_ID)
            S(k,brake_cav_ID+1,3) = dcel_max;
        end
        % Record problem status
        pr_status(k) = pr;
    end
  
    % Update state
    S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
    % Perturbation for the head vehicle
    S(k+1,1,2) = head_vehicle_trajectory.vel(round((k-initialization_time/Tstep+1)/2));
    S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
    
    % update v_star
    v_star = mean(S(k-Tini+1:k,1,2));
    % update past data in control process
    uini = u(:,k-Tini+1:k);
    for k_past = k-Tini+1:k
    % Record output
        y(:,k_past) = measure_mixed_traffic(S(k_past,2:end,2),S(k_past,:,1),ID,v_star,s_star,measure_type);
        e(k_past)   = S(k_past,1,2) - v_star;
    end 
    yini = y(:,k-Tini+1:k);
    eini = S(k-Tini+1:k,1,2) - v_star;
    
    fprintf('Current simulation time: %.2f seconds (%.2f%%) \n',k*Tstep,(k*Tstep-initialization_time)/(total_time--initialization_time)*100);
  
end
k_end = k+1;
y(:,k_end) = measure_mixed_traffic(S(k_end,2:end,2),S(k_end,:,1),ID,v_star,s_star,measure_type);

tsim = toc;

fprintf('Simulation ends at %6.4f seconds \n', tsim);

% -------------------------------------------------------------------------
%   Results output
%--------------------------------------------------------------------------
if mix
switch controller_type
    case 1    
        save(['_data\simulation_data\DeePC\ngsim_simulation\simulation_data',data_str,'_',num2str(i_data),'_ngsim_',num2str(ngsim_id),'_noiseLevel_',num2str(acel_noise),...
            '_hdvType_',num2str(hdv_type),'_lambdaG_',num2str(lambda_g),'_lambdaY_',num2str(lambda_y),'.mat'],...
            'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star');
    case 2
        save(['_data\simulation_data\MPC\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
            '_hdvType_',num2str(hdv_type),'.mat'],...
            'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star');
    case 3
        save(['_data\simulation_data\SPC\constraint_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
            '_hdvType_',num2str(hdv_type),'_lambdaY_',num2str(lambda_y),'.mat'],...
            'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star');
    case 4
        save(['_data\simulation_data\SPC_withoutRegulation\constraint_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
            '_hdvType_',num2str(hdv_type),'_lambdaY_',num2str(lambda_y),'.mat'],...
            'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star');
end
else
    save(['_data\simulation_data\HDV\ngsim_simulation\simulation_data',data_str,'_',num2str(i_data),'_ngsim_',num2str(ngsim_id),'_noiseLevel_',num2str(acel_noise),...
            '_hdvType_',num2str(hdv_type),'.mat'],...
            'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star');

end
end

end
