function [u_opt,y_opt,problem_status] = qp_DeeP_LCC(Up,Yp,Uf,Yf,Ep,Ef,...
    uini,yini,eini,Q,R,r,lambda_g,lambda_y,u_limit,s_limit)
% =========================================================================
%                   DeeP-LCC for mixed traffic
% Input:
%   Up & Uf:             Hankel matrix of pre-collected input data
%   Yp & Yf:             Hankel matrix of pre-collected output data
%   Ep & Ef:             Hankel matrix of pre-collected external input data
%   uini & yini & eini:  past data of length Tini in control process
%   Q & R:               weight coefficient matrix in performance cost
%   r:                   reference trajectory
%   lambda_g & lambda_y: coefficient in regulation for nonlinearty and uncertainty
%   u_limit & s_limit:   bound on control input & spacing
% Output:
%   u_opt:               designed optimal future control input
%   y_opt:               predicted output in the optimal future control input
%   problem_status:      problem status in optimization calculation
%
%                      Optimization Formulation
% mininize:
%   ||y||_{Q_blk}^2 + ||u||_{R_blk}^2 + lambda_g||g||_2^2 + lambda_y||sigma_y||_2^2
% subject to:
%   [Up]    [uini]   [   0   ]
%   [Ep]    [eini]   [   0   ]
%   [Up]g = [yini] + [sigma_y], e = 0, u in u_limit, [0 I_m]y in s_limit
%   [Uf]    [ u  ]   [   0   ]
%   [Ef]    [ e  ]   [   0   ]
%   [Yf]    [ y  ]   [   0   ]
%
% We transform the problem into **standard quadratic programming** for calculation
%
% See Section IV of the following paper for modeling details
%   Title : Data-Driven Predicted Control for Connected and Autonomous
%           Vehicles in Mixed Traffic
%   Author: Jiawei Wang, Yang Zheng, Qing Xu and Keqiang Li
% =========================================================================

% whether there exists input/output constraints
if nargin < 15          
    constraint_bool = 0;
else
    constraint_bool = 1;
end

% ------------
% parameters
% ------------
m        = size(uini,1);                % dimension of control input
p        = size(yini,1);                % dimension of output
Tini     = size(Up,1)/m;                % horizon of past data
N        = size(Uf,1)/m;                % horizon of future data
T        = size(Up,2) + Tini + N - 1;   % time length of pre-collected data

% reshape past data into one single trajectory
% uini = col(u(-Tini),u(-Tini+1),...,u(-1)) (similarly for yini and eini)
uini_col = reshape(uini,[m*Tini,1]);
yini_col = reshape(yini,[p*Tini,1]);
eini_col = reshape(eini,[Tini,1]);
r_col    = reshape(r,[p*N,1]);

Q_blk    = zeros(p*N);
R_blk    = zeros(m*N); 
for i = 1:N
    Q_blk((i-1)*p+1:i*p,(i-1)*p+1:i*p) = Q; 
    R_blk((i-1)*m+1:i*m,(i-1)*m+1:i*m) = R; 
end



% ---------------------
% Standard QP in MATLAB
% [x,fval,exitflag,output,lambda]=quadprog(H,f,A,b,B,c,l,u,x0,options)
% minimize     0.5*x'*H*x+f'*x    
% subject to         A*x          <= b 
%                    B*x           = c
%                    l <= x <= u 
% ---------------------

% Coefficient
H       = Yf'*Q_blk*Yf + Uf'*R_blk*Uf + lambda_g*eye(T-Tini-N+1) + lambda_y*Yp'*Yp;
f       = -lambda_y*Yp'*yini_col;

B       = [Up;Ep;Ef];
c       = [uini_col;eini_col;zeros(N,1)];

if constraint_bool % there exists input/output constraints
    Sf = [zeros(m,p-m),eye(m)];
    Sf_blk = Sf;
    for i = 2:N
        Sf_blk = blkdiag(Sf_blk,Sf); 
    end
    A = [Uf;-Uf;Sf_blk*Yf;-Sf_blk*Yf];
    b = [max(u_limit)*ones(m*N,1);-min(u_limit)*ones(m*N,1);...
        max(s_limit)*ones(m*N,1);-min(s_limit)*ones(m*N,1)];
else
    A = [];
    b = [];
end

options = optimoptions('quadprog','MaxIterations',1e4);
% Optimization
[g_opt,fval,exitflag,output,lambda] = quadprog(H,f,A,b,B,c,[],[],[],options);

% Solution
u_opt   = Uf*g_opt;
y_opt   = Yf*g_opt;
problem_status = exitflag;

% % For infeasible cases
% if exitflag ~= 1
%     u_opt = previous_u_opt;
% end

end