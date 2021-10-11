function [u_opt,y_opt,problem_status] = qpSPC_withExternalInput_withoutRegulation(Up,Yp,Uf,Yf,Ep,Ef,...
    uini,yini,eini,Q,R,r,lambda_y)
% =========================================================================
%               SPC Formulation with Virtual Input
%
% Up & Uf:      collected input data
% Yp & Yf:      collected output data
% Ep & Ef:      collected external input data
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
P   = Yf*pinv([Up;Ep;Yp;Uf;Ef]);
P1  = P(:,1 : m*Tini);
P2  = P(:,m*Tini+1 : m*Tini+Tini);
P3  = P(:,m*Tini+Tini+1 : m*Tini+Tini+p*Tini);
P4  = P(:,m*Tini+Tini+p*Tini+1 : m*Tini+Tini+p*Tini+m*N);

b_y = P1*uini_col + P2*eini_col + P3*yini_col;


% implement SVD on P
SVD_bool = 0;
if SVD_bool
    Pp                      = P(:,1:Tini*(m+1+p));   
    
    [Pp_U,Pp_S,Pp_V]        = svd(Pp);
    Pp_S(find(Pp_S<1e-3))   = 0;

    Pp_hat                  = Pp_U*Pp_S*Pp_V';
    P(:,1:Tini*(m+1+p))     = Pp_hat;
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
H       = P4'*Q_blk*P4 + R_blk;
f       = P4'*Q_blk*b_y;

% Optimization
[u_opt,fval,exitflag,output,lambda] = quadprog(H,f,[],[],[],[]);

% Solution
y_opt       = P4*u_opt+b_y;
problem_status = exitflag;


end