function [u_opt,y_opt,problem_status] = qpDeePC_withExternalInput(Up,Yp,Uf,Yf,Ep,Ef,...
    uini,yini,eini,Q,R,r,lambda_g,lambda_y,u_limit,s_limit)
% =========================================================================
%               DeePC Formulation with Virtual Input
%
% Up & Uf:      collected input data
% Yp & Yf:      collected output data
% Ep & Ef:      collected external input data
% uini & yini:  past data from control process
% Q & R:        penalty for the MPC formulation
% r:            reference trajectory
%
% Objective:    ||(y-r)||_Q + ||u||_R + lambda_g*||g||_2^2 +
%               lambda_y*||sigma_y||_2^2
% =========================================================================


if nargin < 15          % whether there exists input/output constraints
    constraint_bool = 0;
else
    constraint_bool = 1;
end

m        = size(uini,1);
p        = size(yini,1);
Tini     = size(Up,1)/m;
N        = size(Uf,1)/m;
T        = size(Up,2) + Tini + N - 1;


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