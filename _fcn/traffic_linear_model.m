function [Ad,Bd,Cd] = traffic_linear_model(ID,Ts,hdv_type,measure_type,v_star)
% =========================================================================
%                           CLCC model
%
% ID:           vehicle ID
% Ts:           sampling time
% hdv_type:     type of HDV car-following model
% measure_type: measure type
% =========================================================================

switch hdv_type
    case 1
        % Driver Model: OVM
        alpha   = 0.6; 
        beta    = 0.9;
        s_st    = 5;
        s_go    = 35;
        v_max   = 30;
        % Equilibrium spacing
        s_star  = acos(1-v_star/v_max*2)/pi*(s_go-s_st)+s_st; 
        % Linear coefficients
        alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
        alpha2 = alpha+beta;
        alpha3 = beta;
    case 2
        % Driver Model: IDM
        v_max   = 30;
        T_gap   = 1;
        a       = 1;
        b       = 1.5;
        delta   = 4;
        s_st    = 5;
        % Equilibrium spacing
        s_star  = (s_st+T_gap*v_star)/sqrt(1-(v_star/v_max)^delta);
        % Linear coefficients
        alpha1  = 2*a*(s_st+T_gap*v_star)^2/s_star^3;
        alpha2  = sqrt(a/b)*v_star*(s_st+T_gap*v_star)/s_star^2+2*a*(2*v_star^3/v_max^4+T_gap*(s_st+T_gap*v_star)/s_star^2);
        alpha3  = sqrt(a/b)*v_star*(s_st+T_gap*v_star)/s_star^2;
end

pos_cav     = find(ID==1);          % position of CAVs
n_vehicle   = length(ID);           % number of vehicles
n_cav       = length(pos_cav);      % number of CAVs

A = zeros(n_vehicle*2);

P1 = [0,-1;alpha1,-alpha2];
P2 = [0,1;0,alpha3];
S1 = [0,-1;0,0];
S2 = [0,1;0,0];

A(1:2,1:2) = P1;
for i = 2:n_vehicle
    if ID(i) == 0
        A(2*i-1:2*i,2*i-1:2*i)     = P1;
        A(2*i-1:2*i,2*i-3:2*i-2)   = P2;
    else
        A(2*i-1:2*i,2*i-1:2*i)     = S1;
        A(2*i-1:2*i,2*i-3:2*i-2)   = S2;
    end
end

B = zeros(2*n_vehicle,n_cav);
for i = 1:n_cav
   B(pos_cav(i)*2,i) = 1; 
end

switch measure_type
    case 1
        C = zeros(n_vehicle,2*n_vehicle);
        for i = 1:n_vehicle
            C(i,2*i) = 1;
        end
    case 2
        C = zeros(2*n_vehicle);
        for i = 1:n_vehicle
            C(i,2*i) = 1;
            C(n_vehicle+i,2*i-1) = 1;
        end
    case 3
        C = zeros(n_vehicle+n_cav,2*n_vehicle);
        for i = 1:n_vehicle
            C(i,2*i) = 1;
        end
        for i = 1:n_cav
            C(n_vehicle+i,2*pos_cav(i)-1) = 1;
        end
end


Ad      = Ts*A + eye(2*n_vehicle);
Bd      = B*Ts;
Cd      = C; 

end

