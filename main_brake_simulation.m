% =========================================================================
% %                       Safety-critical Simulation
% Scenario:
%       The head vehicle takes a sudden brake
%
% See Section V of the following paper for details
%   Title : Data-Driven Predicted Control for Connected and Autonomous
%           Vehicles in Mixed Traffic
%   Author: Jiawei Wang, Yang Zheng, Qing Xu and Keqiang Li
% =========================================================================

clc; close all; clear;
addpath('_fcn');
warning off;

% -------------------------------------------------------------------------
%   Parameter setup
% -------------------------------------------------------------------------

% ----------------
% Scenario Setup
% ----------------
% whether traffic flow is mixed
mix                 = 1;                    % 0. all HDVs; 1. there exist CAVs
ID                  = [0,0,1,0,0,1,0,0];    % ID of vehicle types
                                            % 1: CAV  0: HDV
pos_cav             = find(ID==1);          % position of CAVs
n_vehicle           = length(ID);           % number of vehicles
n_cav               = length(pos_cav);      % number of CAVs
n_hdv               = n_vehicle-n_cav;      % number of HDVs

% perturbation on the head vehicle
per_type            = 2;    % 1. sinuoid perturbation 2. brake perturbation 
sine_amp            = 5;    % amplitidue of sinuoid perturbation
brake_amp           = 10;   % brake amplitude of brake perturbation

% time
total_time          = 40;              % Total Simulation Time
Tstep               = 0.05;            % Time Step
total_time_step     = total_time/Tstep;

% ----------------
% HDV setup
% ----------------
% Type for HDV car-following model
hdv_type            = 1;    % 1. OVM   2. IDM
% Parameter setup for HDV 
data_str            = '2';  % 1. random ovm  2. manual heterogeneous ovm  3. homogeneous ovm
switch hdv_type
    case 1
        load(['_data/hdv_ovm_',data_str,'.mat']);
    case 2
        load('_data/hdv_idm.mat');      
end
% Uncertainty for HDV acceleration
acel_noise          = 0.1;  % A white noise signal on HDV's acceleration

% -------------------------------------------------------------------------
%   Formulation for DeeP-LCC
% ------------------------------------------------------------------------- 
% ----------------
% Parameter setup
% ----------------
% Type of the controller
controller_type     = 2;    % 1. DeeP-LCC  2. MPC 
% Initialize Equilibrium Setup (they might be updated in the control process)
v_star              = 15;   % Equilibrium velocity
s_star              = 20;   % Equilibrium spacing for CAV
% Horizon setup 
Tini                = 20;   % length of past data in control process
N                   = 50;   % length of future data in control process
% Performance cost
weight_v            = 1;    % weight coefficient for velocity error
weight_s            = 0.5;  % weight coefficient for spacing error   
weight_u            = 0.1;  % weight coefficient for control input
% Setup in DeeP-LCC
T                   = 2000; % length of data samples
lambda_g            = 100;  % penalty on ||g||_2^2 in objective
lambda_y            = 1e4;  % penalty on ||sigma_y||_2^2 in objective
% Constraints
constraint_bool     = 1;    % whether there exist constraints
acel_max            = 2;    % maximum acceleration
dcel_max            = -5;   % minimum acceleration (maximum deceleration)
spacing_max         = 40;   % maximum spacing
spacing_min         = 5;    % minimum spacing
u_limit             = [dcel_max,acel_max];
s_limit             = [spacing_min,spacing_max]-s_star;
% what signals are measurable (for output definition)
measure_type        = 3;    % 1. Only the velocity errors of all the vehicles are measurable;
                            % 2. All the states, including velocity error and spacing error are measurable;
                            % 3. Velocity error and spacing error of the CAVs are measurable, 
                            %    and the velocity error of the HDVs are measurable.
% equilibrium setup
fixed_spacing_bool      = 0;    % wheter fix equilibrium spacing
                                % 0. the equilibrium spacing will be updated via an OVM-type spacing policy
                                % 1. fix the equilibrium spacing


% ----------------
% Process parameters
% ----------------
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

Q_v         = weight_v*eye(n_vehicle);          % penalty for velocity error
Q_s         = weight_s*eye(p_ctr-n_vehicle);    % penalty for spacing error
Q           = blkdiag(Q_v,Q_s);                 % penalty for trajectory error
R           = weight_u*eye(m_ctr);              % penalty for control input

u           = zeros(m_ctr,total_time_step);     % control input
x           = zeros(n_ctr,total_time_step);     % state variables
y           = zeros(p_ctr,total_time_step);     % output variables
pr_status   = zeros(total_time_step,1);         % problem status
e           = zeros(1,total_time_step);         % external input

% ----------------
% Pre-collected data
% ----------------
% load pre-collected data for DeeP-LCC
i_data              = 1;    % id of the pre-collected data
load(['_data\trajectory_data_collection\data',data_str,'_',num2str(i_data),'_noiseLevel_',num2str(acel_noise),'.mat']);

% -------------------------------------------------------------------------
%   Simulation
%--------------------------------------------------------------------------
% Mixed traffic states
% S(time,vehicle id,state variable), in state variable: 1. position; 2. velocity; 3. acceleration
S               = zeros(total_time_step,n_vehicle+1,3); 
S(1,1,1)        = 0;
for i = 2 : n_vehicle+1
    S(1,i,1)    = S(1,i-1,1) - hdv_parameter.s_star(i-1);
end
S(1,:,2)        = v_star * ones(n_vehicle+1,1);

%  reference trajectory is all zeros: stabilize the system to equilibrium
r               = zeros(p_ctr,total_time_step+N); 

% -------------------------------------------------------------------------
%   Experiment starts here
%--------------------------------------------------------------------------
tic

% ------------------
%  Initialization: the CAVs and the head vehicle have zero control input
% ------------------
% initial past data in control process
uini = zeros(m_ctr,Tini);
eini = zeros(1,Tini);
yini = zeros(p_ctr,Tini);

for k = 1:Tini-1
    % calculate acceleration for the HDVs
    acel                =  HDV_dynamics(S(k,:,:),hdv_parameter) ...
                                -acel_noise + 2*acel_noise*rand(n_vehicle,1);
    
    S(k,1,3)           = 0;               % the head vehicle
    S(k,2:end,3)       = acel;            % all the vehicles use HDV model
    S(k,pos_cav+1,3)   = uini(:,k);       % the CAV
    
    % update traffic states
    S(k+1,:,2)          = S(k,:,2) + Tstep*S(k,:,3);
    S(k+1,1,2)          = eini(k) + v_star;          % the velocity of the head vehicle
    S(k+1,:,1)          = S(k,:,1) + Tstep*S(k,:,2);
    
    % update past output data
    yini(:,k)           = measure_mixed_traffic(S(k,2:end,2),S(k,:,1),ID,v_star,s_star,measure_type);
    
end

k_end = k+1;
yini(:,k_end) = measure_mixed_traffic(S(k_end,2:end,2),S(k_end,:,1),ID,v_star,s_star,measure_type);

% update data in u,e,y
u(:,1:Tini) = uini;
e(:,1:Tini) = eini;
y(:,1:Tini) = yini;

% For MPC, which might have infeasible cases
previous_u_opt = 0; 

% ------------------
%  Continue the simulation
% ------------------
for k = Tini:total_time_step-1
    % calculate acceleration for the HDVs
    acel         =  HDV_dynamics(S(k,:,:),hdv_parameter) ...
                    -acel_noise + 2*acel_noise*rand(n_vehicle,1);
    S(k,2:end,3) = acel;     % all the vehicles use HDV model

    
    if mix
        switch controller_type
            case 1  % calculate control input via DeeP-LCC     
                if constraint_bool
                    [u_opt,y_opt,pr] = qp_DeeP_LCC(Up,Yp,Uf,Yf,Ep,Ef,uini,yini,eini,Q,R,r(:,k:k+N-1),...
                        lambda_g,lambda_y,u_limit,s_limit);
                else
                    [u_opt,y_opt,pr] = qp_DeeP_LCC(Up,Yp,Uf,Yf,Ep,Ef,uini,yini,eini,Q,R,r(:,k:k+N-1),...
                        lambda_g,lambda_y);
                end
            case 2  % calculate control input via MPC
                if constraint_bool
                    [u_opt,y_opt,pr] = qp_MPC(ID,Tstep,hdv_type,measure_type,v_star,uini,yini,N,Q,R,r(:,k:k+N-1),u_limit,s_limit,previous_u_opt);
                    previous_u_opt   = u_opt;
                else
                    [u_opt,y_opt,pr] = qp_MPC(ID,Tstep,hdv_type,measure_type,v_star,uini,yini,N,Q,R,r(:,k:k+N-1));                    
                end
         end
        % one-step implementation in receding horizon manner
        u(:,k) = u_opt(1:m_ctr,1);
        % update accleration for the CAV
        S(k,pos_cav+1,3)   = u(:,k);
        % judge whether AEB (automatic emergency braking, which is implemented in the function of 'HDV_dynamics') commands to brake
        brake_vehicle_ID = find(acel==dcel_max);                % the vehicles that need to brake
        brake_cav_ID     = intersect(brake_vehicle_ID,pos_cav); % the CAVs that need to brake
        if ~isempty(brake_cav_ID)
            S(k,brake_cav_ID+1,3) = dcel_max;
        end
        % record problem status
        pr_status(k) = pr;
    end
  
    % update traffic states
    S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
    % perturbation for the head vehicle
    switch per_type
        case 1
            S(k+1,1,2) = v_star + sine_amp*sin(2*pi/(10/Tstep)*(k-Tini));
            S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
        case 2
            if (k-Tini)*Tstep < brake_amp/5
                S(k+1,1,3) = -5;
            elseif (k-Tini)*Tstep < brake_amp/5+5
                S(k+1,1,3) = 0;
            elseif (k-Tini)*Tstep < brake_amp/5+5+5
                S(k+1,1,3) = 2;
            else
                S(k+1,1,3) = 0;
            end
            S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
            S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
    end
    
    % update equilibrium setup for the CAVs
    v_star = mean(S(k-Tini+1:k,1,2));               % update v_star
    if ~fixed_spacing_bool        
        s_star = acos(1-v_star/30*2)/pi*(35-5) + 5; % update s_star
        
    end
    
    % update past data in control process
    uini = u(:,k-Tini+1:k);
    % the output needs to be re-calculated since the equilibrium might have been updated
    for k_past = k-Tini+1:k
    % Record output
        y(:,k_past) = measure_mixed_traffic(S(k_past,2:end,2),S(k_past,:,1),ID,v_star,s_star,measure_type);
        e(k_past)   = S(k_past,1,2) - v_star;
    end 
    yini = y(:,k-Tini+1:k);
    eini = S(k-Tini+1:k,1,2) - v_star;
  
    fprintf('Simulation number: %d  |  process... %2.2f%% \n',i_data,(k-Tini)/total_time_step*100);
    %fprintf('Fixed Spacing: %d',fixed_spacing_bool);
    %fprintf('Current spacing of the first CAV: %4.2f \n',S(k,3,1)-S(k,4,1));
  
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
        save(['..\_data\simulation_data\DeeP_LCC\constrained_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
            '_fixSpacing_',num2str(fixed_spacing_bool),...
            '_hdvType_',num2str(hdv_type),'_lambdaG_',num2str(lambda_g),'_lambdaY_',num2str(lambda_y),'.mat'],...
            'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star','pr_status');

    case 2
        save(['..\_data\simulation_data\MPC\constrained_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
            '_fixSpacing_',num2str(fixed_spacing_bool),...
            '_hdvType_',num2str(hdv_type),'.mat'],...
            'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star','pr_status');
end
else
    save(['..\_data\simulation_data\HDV\constrained_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
            '_hdvType_',num2str(hdv_type),'.mat'],...
            'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star');
end


