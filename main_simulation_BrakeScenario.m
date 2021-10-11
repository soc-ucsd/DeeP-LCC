% =========================================================================
%               Traffic Simulation with Already Collected Data
%               Scenario: general case with multiple CAVs and multiple HDVs
%               The head vehicle has a brake perturbation
% =========================================================================

clc; close all; clear;
addpath('_fcn');
warning off;

% -------------------------------------------------------------------------
%   Parameter setup
% -------------------------------------------------------------------------

% Data number
data_number     = 1;
% Data set
data_str        = '2';  % 1. random ovm  2. manual ovm  3. homogeneous ovm
% Perturbation amplitude
per_type         = 2; % 1. sinuoid perturbation 2. brake perturbation 
sine_amp         = 5; % amplitidue of sinuoid perturbation
brake_amp        = 10;% brake amplitude of brake perturbation
% Whether there exist constraints
constraint_bool  = 1;
% Wheter fix equilibrium state
fixed_equilibrium_bool = 1;
% Wheter fix desired spacing
fixed_spacing_bool = 0;

% Type of the controller
controller_type = 1;    % 1. DeePC  2. MPC  3.SPC  4. SPC without Regulation

% Type for HDV car-following model
hdv_type        = 1;    % 1. OVM   2. IDM
% Uncertainty for HDV behavior
acel_noise      = 0.1;    % A white noise signal on HDV's original acceleration

% Parameters in Simulation
total_time       = 40;              % Total Simulation Time
Tstep            = 0.05;            % Time Step
total_time_step  = total_time/Tstep;

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

mix         = 1;                    % whether mixed traffic flow

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
%  Initial trajectory
% ------------------
uini = zeros(m_ctr,Tini);
eini = zeros(1,Tini);
yini = zeros(p_ctr,Tini);

for k = 1:Tini-1
    % Update acceleration
    acel         =  HDV_dynamics(S(k,:,:),hdv_parameter) ...
                    -acel_noise + 2*acel_noise*rand(n_vehicle,1);
    
    S(k,1,3)           = 0;               % the head vehicle
    S(k,2:end,3)       = acel;            % all the vehicles using HDV model
    S(k,pos_cav+1,3)   = uini(:,k);       % the CAV
    
    S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
    S(k+1,1,2) = eini(k) + v_star;          % the velocity of the head vehicle
    S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
    
    yini(:,k) = measure_mixed_traffic(S(k,2:end,2),S(k,:,1),ID,v_star,s_star,measure_type);
    
end

k_end = k+1;
yini(:,k_end) = measure_mixed_traffic(S(k_end,2:end,2),S(k_end,:,1),ID,v_star,s_star,measure_type);


u(:,1:Tini) = uini;
e(:,1:Tini) = eini;
y(:,1:Tini) = yini;

% For MPC and DeePC with constraints
previous_u_opt = 0; 

% ------------------
%  simulation starts here
% ------------------
for k = Tini:total_time_step-1
    % Update acceleration
    acel         =  HDV_dynamics(S(k,:,:),hdv_parameter) ...
                    -acel_noise + 2*acel_noise*rand(n_vehicle,1);
    
    S(k,2:end,3) = acel;     % all the vehicles using HDV model
    if min(min(yini)) < -15
       temp = 1; 
    end
    
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
                if constraint_bool
                    [u_opt,y_opt,pr] = qpMPC(ID,Tstep,hdv_type,measure_type,v_star,uini,yini,N,Q,R,r(:,k:k+N-1),u_limit,s_limit,previous_u_opt);
                    previous_u_opt   = u_opt;
                else
                    [u_opt,y_opt,pr] = qpMPC(ID,Tstep,hdv_type,measure_type,v_star,uini,yini,N,Q,R,r(:,k:k+N-1));                    
                end
%                 if pr~=1
%                     break;
%                 end
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
    
    if ~fixed_equilibrium_bool
        % update v_star
        v_star = mean(S(k-Tini+1:k,1,2));
        if ~fixed_spacing_bool
            s_star = acos(1-v_star/30*2)/pi*(35-5) + 5;
        else
        end
    end
    % update past data in control process
    uini = u(:,k-Tini+1:k);
    for k_past = k-Tini+1:k
    % Record output
        y(:,k_past) = measure_mixed_traffic(S(k_past,2:end,2),S(k_past,:,1),ID,v_star,s_star,measure_type);
        e(k_past)   = S(k_past,1,2) - v_star;
    end 
    yini = y(:,k-Tini+1:k);
    eini = S(k-Tini+1:k,1,2) - v_star;
    
    
    
    fprintf('Simulation number: %d  |  process... %2.2f%% \n',i_data,(k-Tini)/total_time_step*100);
    fprintf('Fixed Spacing: %d',fixed_spacing_bool);
    fprintf('Current spacing of the first CAV: %4.2f \n',S(k,3,1)-S(k,4,1));
  
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
        if fixed_equilibrium_bool
            save(['_data\simulation_data\DeePC\constrained_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
                '_fixedEquilibrium',...
                '_hdvType_',num2str(hdv_type),'_lambdaG_',num2str(lambda_g),'_lambdaY_',num2str(lambda_y),'.mat'],...
                'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star','pr_status');
        else
            save(['_data\simulation_data\DeePC\constrained_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
                '_fixSpacing_',num2str(fixed_spacing_bool),...
                '_hdvType_',num2str(hdv_type),'_lambdaG_',num2str(lambda_g),'_lambdaY_',num2str(lambda_y),'.mat'],...
                'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star','pr_status');
        end
    case 2
        if fixed_equilibrium_bool
            save(['_data\simulation_data\MPC\constrained_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
                '_fixedEquilibrium',...
                '_hdvType_',num2str(hdv_type),'.mat'],...
                'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star','pr_status');
        else
            save(['_data\simulation_data\MPC\constrained_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
                '_fixSpacing_',num2str(fixed_spacing_bool),...
                '_hdvType_',num2str(hdv_type),'.mat'],...
                'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star','pr_status');
        end
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
    save(['_data\simulation_data\HDV\constrained_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
            '_hdvType_',num2str(hdv_type),'.mat'],...
            'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star');
end
end

