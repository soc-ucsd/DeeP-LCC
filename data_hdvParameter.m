% =========================================================================
%               Generate heterogeneous HDV paramters
% =========================================================================

clc; close all; clear;

hdv_type = 1;
v_star   = 15;

% -------------------
% ID          = [0,0,1,0,0,1,0,0];    % ID of vehicle types
                                    % 1: CAV  0: HDV
% -------------------

% Homegeneous setup for OVM
alpha       = 0.6*ones(8,1);
beta        = 0.9*ones(8,1);
s_go        = 25*ones(8,1);
s_st        = 5;
v_max       = 30;
% Equilibrium spacing
s_star  = acos(1-v_star/v_max*2)./pi*(s_go-s_st) + s_st;

hdv_parameter = struct('type',hdv_type,...
            'alpha',alpha,'beta',beta,'s_st',s_st,'s_go',s_go,'v_max',v_max,'s_star',s_star);
save('_data/hdv_ovm_homogeneous.mat','hdv_parameter');

switch hdv_type
    case 1
        % Driver Model: OVM
        alpha       = 0.4 + 0.4*rand(8,1);
        beta        = 0.7 + 0.4*rand(8,1);
        s_go        = 30 + 10*rand(8,1);
        s_st        = 5;
        v_max       = 30;
        
        % Manual set for parameters        
        alpha   = [0.45;0.75;0.60;0.70;0.50;0.60;0.40;0.80];
        beta    = [0.60;0.95;0.90;0.95;0.75;0.90;0.80;1.00];
        s_go    = [ 38 ; 31 ; 35 ; 33 ; 37 ; 35 ; 39 ; 34 ];
        
        % Consider nominal parameter for the CAV position, which only works
        % in comparison for all the vehicles are HDVs
        alpha(3)    = 0.6;
        alpha(6)    = 0.6;
        beta(3)     = 0.9;
        beta(6)     = 0.9;
        s_go(3)     = 35;
        s_go(6)     = 35;
        
        % Equilibrium spacing
        s_star  = acos(1-v_star/v_max*2)./pi*(s_go-s_st) + s_st;
        
    case 2
        % Driver Model: IDM
        v_max       = 30;
        T_gap       = ones(8,1);
        a           = 1;
        b           = 1.5;
        delta       = 4;
        s_st        = 5;
        
        % Equilibrium spacing
        s_star  = (s_st+T_gap.*v_star)./sqrt(1-(v_star/v_max)^delta);
end

switch hdv_type
    case 1
        hdv_parameter = struct('type',hdv_type,...
            'alpha',alpha,'beta',beta,'s_st',s_st,'s_go',s_go,'v_max',v_max,'s_star',s_star);
        save('_data/hdv_ovm.mat','hdv_parameter');
    case 2
        hdv_parameter = struct('type',hdv_type,...
            'v_max',v_max,'T_gap',T_gap,'a',a,'b',b,'delta',delta,'s_st',s_st,'s_star',s_star);
        save('_data/hdv_idm.mat','hdv_parameter');
end