function [u_opt,y_opt,problem_status] = qp_MPC(ID,Ts,hdv_type,measure_type,...
    v_star,uini,yini,N,Q,R,r,u_limit,s_limit,previous_u_opt)
% =========================================================================
%                           MPC for mixed traffic
% Input:
%   ID:                 vehicle ID
%   Ts:                 sampling time
%   hdv_type:           type of HDV car-following model
%   measure_type:       measure type for output definition
%   v_star:             equilibrium velocity
%   uini & yini:        past data of length Tini in control process
%   N:                  future time length (predicted horizon)
%   Q & R:              weight coefficient matrix in performance cost
%   r:                  reference trajectory
%   u_limit & s_limit:  bound on control input & spacing
%   previous_u_opt:     control input in previous time step
% Output:
%   u_opt:               designed optimal future control input
%   y_opt:               predicted output in the optimal future control input
%   problem_status:      problem status in optimization calculation
%
%                      Optimization Formulation
% mininize:
%   ||y||_{Q_blk}^2 + ||u||_{R_blk}^2
% subject to:
%   xini is estimated from past data uini,yini 
%   x    = Ax + Bu
%   y    = Cx
% 
% We transform the problem into **standard quadratic programming** for calculation
%
% See Section IV of the following paper for modeling details
%   Title : Data-Driven Predicted Control for Connected and Autonomous
%           Vehicles in Mixed Traffic
%   Author: Jiawei Wang, Yang Zheng, Qing Xu and Keqiang Li
% =========================================================================


% -------------------------------------------------------------------------
%  Calculate via Linear model
% -------------------------------------------------------------------------
if nargin < 12          % whether there exists input/output constraints
    constraint_bool = 0;
else
    constraint_bool = 1;
end

%  linear model
[A,B,C]     = traffic_linear_model(ID,Ts,hdv_type,measure_type,v_star);
%  dimension
m           = size(uini,1);     % dimension of control input
p           = size(yini,1);     % dimension of output
n           = size(A,1);        % dimension of state
Tini        = size(uini,2);     % horizon of past data


% reshape past data into one single trajectory
% uini = col(u(-Tini),u(-Tini+1),...,u(-1)) (similarly for yini and eini)
uini_col    = reshape(uini,[m*Tini,1]);
yini_col    = reshape(yini,[p*Tini,1]);

Obsv_Tini = zeros(p*Tini,n);
for i = 1:Tini
    Obsv_Tini((i-1)*p+1:i*p,:) = C*A^(i-1);
end

Toep_Tini = zeros(p*Tini,m*Tini);
Toep_Tini(1*p+1:2*p,0*m+1:1*m) = C*B;
for i = 3:Tini
    for j = 1:i-1
        Toep_Tini((i-1)*p+1:i*p,(j-1)*m+1:j*m) = C*A^(i-1-j)*B;
    end
end

Obsv_N = zeros(p*N,n);
for i = 1:N
    Obsv_N((i-1)*p+1:i*p,:) = C*A^(i-1);
end

Toep_N = zeros(p*N,m*N);
Toep_N(1*p+1:2*p,0*m+1:1*m) = C*B;
for i = 3:N
    for j = 1:i-1
        Toep_N((i-1)*p+1:i*p,(j-1)*m+1:j*m) = C*A^(i-1-j)*B;
    end
end

Ctrb_Tini = zeros(n,m*Tini);
for i = 1:Tini
   Ctrb_Tini(:,(i-1)*m+1:i*m) = A^(Tini-i)*B; 
end

x_1             = pinv(Obsv_Tini)*(yini_col-Toep_Tini*uini_col);
x_Tini_plus1    = A^Tini*x_1 + Ctrb_Tini*uini_col;

r_col    = reshape(r,[p*N,1]);


Q_blk    = zeros(p*N);
R_blk    = zeros(m*N); 
for i = 1:N
    Q_blk((i-1)*p+1:i*p,(i-1)*p+1:i*p) = Q; 
    R_blk((i-1)*m+1:i*m,(i-1)*m+1:i*m) = R; 
end

Cu       = Obsv_N*(Ctrb_Tini-A^Tini*pinv(Obsv_Tini)*Toep_Tini)*uini_col + Obsv_N*A^Tini*pinv(Obsv_Tini)*yini_col;
% ---------------------
% Standard QP in MATLAB
% [x,fval,exitflag,output,lambda]=quadprog(H,f,A,b,B,c,l,u,x0,options)
% minimize     0.5*x'*H*x+f'*x    
% subject to         A*x          <= b 
%                    B*x           = c
%                    l <= x <= u 
% ---------------------


% Coefficient
H       = Toep_N'*Q_blk*Toep_N+R_blk;
f       = Toep_N'*Q_blk*Obsv_N*x_Tini_plus1;

if constraint_bool % there exists input/output constraints
    Sf = [zeros(m,p-m),eye(m)];
    Sf_blk = Sf;
    for i = 2:N
        Sf_blk = blkdiag(Sf_blk,Sf); 
    end
    A = [eye(m*N);-eye(m*N);...
        Sf_blk*Toep_N;-Sf_blk*Toep_N];
    b = [max(u_limit)*ones(m*N,1);-min(u_limit)*ones(m*N,1);...
        max(s_limit)*ones(m*N,1)-Sf_blk*Cu;-min(s_limit)*ones(m*N,1)+Sf_blk*Cu];
else
    A = [];
    b = [];
end

% Optimization
[u_opt,fval,exitflag,output,lambda] = quadprog(H,f,A,b,[],[]);

% For infeasible cases
if exitflag ~= 1
    u_opt = previous_u_opt;
end

% Solution
y_opt       = Obsv_N*x_Tini_plus1+Toep_N*u_opt;
problem_status = exitflag;



end