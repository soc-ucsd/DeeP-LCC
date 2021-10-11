function [u_opt,y_opt,problem_status] = SPC_withVirtualInput(Up,Yp,Uf,Yf,Ep,Ef,...
    uini,yini,eini,Q,R,r,lambda_y)
% =========================================================================
%               SPC Formulation with Virtual Input
%
% Up & Uf:      collected input data
% Yp & Yf:      collected output data
% uini & yini:  past data from control process
% Q & R:        penalty for the MPC formulation
% r:            reference trajectory
%
% Objective:    ||(y-r)||_Q + ||u||_R
% =========================================================================

if nargin < 14          % whether there exists input constraints
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
eini_col = reshape(eini,[Tini,1]);
r_col    = reshape(r,[p*N,1]);

m_virtual = size(Ep,1)/Tini;

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
P = Yf*pinv([Up;Ep;Yp;Uf;Ef]);

% implement SVD on P
SVD_bool = 0;
if SVD_bool
    Pp                      = P(:,1:Tini*(m+1+p));   
    
    [Pp_U,Pp_S,Pp_V]        = svd(Pp);
    Pp_S(find(Pp_S<1e-3))   = 0;

    Pp_hat                  = Pp_U*Pp_S*Pp_V';
    P(:,1:Tini*(m+1+p))     = Pp_hat;
end


% ------------------
%  variables
% ------------------
u       = sdpvar(m*N,1);
y       = sdpvar(p*N,1);
sigma_y = sdpvar(p*Tini,1);

% -------------------------------------------------------------------------
%  optimization formulation
% -------------------------------------------------------------------------

% ------------------
% Fundamental lemma
% ------------------

e    = zeros(m_virtual*N,1);

constraints  = [y == P*[uini_col;eini_col;yini_col+sigma_y;u;e]];



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

objective   = (y-r_col)'*Q_blk*(y-r_col) + u'*R_blk*u ...
                +lambda_y*norm(sigma_y,1);


opts        = sdpsettings('solver','mosek');
sol         = optimize(constraints,objective,opts);

% ------------------
%  optimization result
% ------------------
u_opt          = value(u);
y_opt          = value(y);
problem_status = sol.problem;

end