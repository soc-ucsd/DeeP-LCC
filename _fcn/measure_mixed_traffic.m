function [y] = measure_mixed_traffic(vel,pos,ID,v_star,s_star,type)
% =========================================================================
%               Measure the output in mixed traffic flow
%
% vel:      velocity of each vehicle
% pos:      position of each vehicle    
% ID:       ID of vehicle types     1: CAV  0: HDV
% v_star:   equilibrium velocity
% s_star:   equilibrium spacing
% type:
% 1. Only the velocity errors of all the vehicles are measurable;
% 2. All the states, including velocity error and spacing error are measurable;
% 3. Velocity error and spacing error of the CAVs are measurable, 
%    and the velocity error of the HDVs are measurable.
% =========================================================================

pos_cav     = find(ID==1);          % position of CAVs

switch type
    case 1
        y = (vel - v_star)';
    case 2
        spacing = pos(1:end-1) - pos(2:end);
        y = [(vel - v_star)';
             (spacing - s_star)'];
    case 3
        spacing = pos(1:end-1) - pos(2:end);
        y = [(vel - v_star)';
             (spacing(pos_cav) - s_star)'];
end

end

