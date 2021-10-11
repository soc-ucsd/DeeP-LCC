function [u_opt,y_opt,problem_status] = SPC(Up,Yp,Uf,Yf,uini,yini,Q,R,r)
% =========================================================================
%               SPC Formulation
%
% Up & Uf:      collected input data
% Yp & Yf:      collected output data
% uini & yini:  past data from control process
% Q & R:        penalty for the MPC formulation
% r:            reference trajectory
%
% Objective:    ||(y-r)||_Q + ||u||_R
% =========================================================================

if nargin < 10          % whether there exists input constraints
    u_limit_bool = 0;
else
    u_limit_bool = 1;
end

m       = size(uini,1);
p       = size(yini,1);
Tini    = size(Up,1)/m;
N       = size(Uf,1)/m;
T       = size(Up,2) + Tini + N - 1;

uini_col = reshape(uini,[m*Tini,1]);
yini_col = reshape(yini,[p*Tini,1]);
r_col    = reshape(r,[p*N,1]);


Q_blk    = zeros(p*N);
R_blk    = zeros(m*N); 
for i = 1:N
    Q_blk((i-1)*p+1:i*p,(i-1)*p+1:i*p) = Q; 
    R_blk((i-1)*m+1:i*m,(i-1)*m+1:i*m) = R; 
end

% Q_blk   = Q;
% R_blk   = R;
% for i = 2:N
%     Q_blk = blkdiag(Q_blk,Q); 
%     R_blk = blkdiag(R_blk,R); 
% end

% ------------------
%  Calculate the Predictor
% ------------------
P = Yf*pinv([Up;Yp;Uf]);

% ------------------
%  variables
% ------------------
u       = sdpvar(m*N,1);
y       = sdpvar(p*N,1);

% -------------------------------------------------------------------------
%  optimization formulation
% -------------------------------------------------------------------------

% ------------------
% Fundamental lemma
% ------------------


constraints  = [y == P*[uini_col;yini_col;u]];


% ------------------
% Input constraints
% ------------------
if u_limit_bool
    for i=1:m*N
        constraints = [constraints, u(i)<=u_limit(1), u(i)>=u_limit(2)];
    end
end
% ------------------
% Objective function
% ------------------

objective   = (y-r_col)'*Q_blk*(y-r_col) + u'*R_blk*u;


opts        = sdpsettings('solver','mosek');
sol         = optimize(constraints,objective,opts);

% ------------------
%  optimization result
% ------------------
u_opt          = value(u);
y_opt          = value(y);
problem_status = sol.problem;

end