% =========================================================================
%               Data collection for random times
% =========================================================================

clc; close all; clear;
addpath('_fcn');

data_total_number = 100;

h_wait = waitbar(0,'please wait');

for i_data = 1:data_total_number

% -------------------------------------------------------------------------
%   Parameter setup
% -------------------------------------------------------------------------

% Type for HDV car-following model
hdv_type        = 1;    % 1. OVM   2. IDM
% Uncertainty for HDV behavior
acel_noise      = 0.1;  % A white noise signal on HDV's original acceleration
% Data set
data_str        = '3';  % 1. random ovm  2. manual ovm  3. homogeneous ovm

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

lambda_g     = 1;        % penalty on ||g||_2^2 in objective
lambda_y     = 1e3;      % penalty on ||sigma_y||_2^2 in objective

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

switch hdv_type
    case 1
        % Driver Model: OVM
        load(['_data/hdv_ovm_',num2str(data_str),'.mat']);
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

acel_max = 2;
dcel_max = -5;
    
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

% -------------------------------------------------------------------------
%   Data collection
%-------------------------------------------------------------------------- 

% ------------------
%  persistently exciting input data
% ------------------
ud          = -1+2*rand(m_ctr,T);
ed          = -1+2*rand(1,T);
yd          = zeros(p_ctr,T);

% ------------------
%  generate output data
% ------------------
for k = 1:T-1
    % Update acceleration
    acel               = HDV_dynamics(S(k,:,:),hdv_parameter) ...
                         -acel_noise + 2*acel_noise*rand(n_vehicle,1);
    
    S(k,1,3)           = 0;         % the head vehicle
    S(k,2:end,3)       = acel;      % all the vehicles using HDV model
    S(k,pos_cav+1,3)   = ud(:,k);   % the CAVs
    
    S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
    S(k+1,1,2) = ed(k) + v_star;   % the velocity of the head vehicle
    S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);    
    
    yd(:,k) = measure_mixed_traffic(S(k,2:end,2),S(k,:,1),ID,v_star,s_star,measure_type);
end
k = k+1;
yd(:,k) = measure_mixed_traffic(S(k,2:end,2),S(k,:,1),ID,v_star,s_star,measure_type);

% ------------------
%  organize past data and future data
% ------------------
U   = hankel_matrix(ud,Tini+N);
Up  = U(1:Tini*m_ctr,:);
Uf  = U((Tini*m_ctr+1):end,:);

E   = hankel_matrix(ed,Tini+N);
Ep  = E(1:Tini,:);
Ef  = E((Tini+1):end,:);

Y   = hankel_matrix(yd,Tini+N);
Yp  = Y(1:Tini*p_ctr,:);
Yf  = Y((Tini*p_ctr+1):end,:);

str=['Processing...',num2str(i_data/data_total_number*100),'%'];
    waitbar(i_data/data_total_number,h_wait,str);

save(['_data\trajectory_data_collection\data',num2str(data_str),'_',num2str(i_data),'_noiseLevel_',num2str(acel_noise),'.mat'],...
    'hdv_type','acel_noise','Up','Yp','Uf','Yf','Ep','Ef','T','Tini','N','ID','Tstep','v_star');

end

close(h_wait);