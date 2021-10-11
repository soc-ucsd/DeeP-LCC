function [acel] = HDV_dynamics(S,parameter)
% =========================================================================
%               Calculate the acceleration of HDVs
% S:            state of all the vehicles
% type:         type of the HDV car-following model
% parameter:    Parameter value in the car-following model
% =========================================================================

num_vehicle = size(S,2)-1;

acel_max    = 2;
dcel_max    = -5;

switch parameter.type
    case 1                    
        V_diff = S(1,1:(end-1),2) - S(1,2:end,2);
        D_diff = S(1,1:(end-1),1) - S(1,2:end,1);
        cal_D = D_diff'; % For the boundary of Optimal Veloicity Calculation
        for i = 1:num_vehicle
            if cal_D(i) > parameter.s_go(i)
                cal_D(i) = parameter.s_go(i);
            elseif cal_D(i) < parameter.s_st
                cal_D(i) = parameter.s_st;
            end
        end        
        
        % nonlinear OVM Model
        % V_d = v_max/2*(1-cos(pi*(s-s_st)/(s_go-s_st)));
        % a   = alpha*(V_d-v2)+beta*(v1-v2);
        acel  = parameter.alpha.*(parameter.v_max/2.*(1-cos(pi*(cal_D-parameter.s_st)./(parameter.s_go-parameter.s_st))) ...
                - S(1,2:end,2)') + parameter.beta.*V_diff';       
        
        % % acceleration saturation
        acel(acel>acel_max) = acel_max;
        acel(acel<dcel_max) = dcel_max;
        %
        % SD as ADAS to prevent crash
        acel_sd = (S(1,2:end,2).^2-S(1,1:(end-1),2).^2)./2./D_diff;
        acel(acel_sd>abs(dcel_max)) = dcel_max;
    case 2
        % Driver Model: IDM
        v_max       = 30;
        T_gap       = 1;
        a           = 1;
        b           = 1.5;
        delta       = 4;
        s_st        = 5;
        
        V_diff  = S(1,1:(end-1),2) - S(1,2:end,2);
        D_diff  = S(1,1:(end-1),1) - S(1,2:end,1);
        
        acel    = a.*(1- (S(1,2:end,2)/v_max).^delta -...
                    ((s_st+T_gap.*S(1,2:end,2)-V_diff.*S(1,2:end,2)/2./sqrt(a)./sqrt(b))./D_diff).^2);
        acel    = acel'; 
        % % acceleration saturation
        % acel(acel>acel_max) = acel_max;
        % acel(acel<dcel_max) = dcel_max;
        %
        % SD as ADAS to prevent crash
        acel_sd = (S(1,2:end,2).^2-S(1,1:(end-1),2).^2)./2./D_diff;
        acel(acel_sd>abs(dcel_max)) = dcel_max;
end

end

