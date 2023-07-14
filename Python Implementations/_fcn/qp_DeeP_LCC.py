import numpy as np
from cvxopt import solvers
from cvxopt import matrix
#import scipy.io as spio  #For debugging

# For debugging
# mat = spio.loadmat('data.mat', squeeze_me=True)
# Up = np.matrix(mat['Up']);
# Yp = np.matrix(mat['Yp']);
# Uf = np.matrix(mat['Uf']);
# Yf = np.matrix(mat['Yf']);
# Ep = np.matrix(mat['Ep']);
# Ef = np.matrix(mat['Ef']);
# uini = np.matrix(mat['uini']);
# yini = np.matrix(mat['yini']);
# eini = np.matrix(mat['eini']);
# Q = np.matrix(mat['Q']);
# R = np.matrix(mat['R']);
# r = np.matrix(mat['r']);
# lambda_g = np.matrix(mat['lambda_g']);
# lambda_y = np.matrix(mat['lambda_y']);
# u_limit = np.matrix(mat['u_limit']);
# s_limit = np.matrix(mat['s_limit']);

# =========================================================================
#                   DeeP-LCC for mixed traffic
# Input:
#   Up & Uf:             Hankel matrix of pre-collected input data
#   Yp & Yf:             Hankel matrix of pre-collected output data
#   Ep & Ef:             Hankel matrix of pre-collected external input data
#   uini & yini & eini:  past data of length Tini in control process
#   Q & R:               weight coefficient matrix in performance cost
#   r:                   reference trajectory
#   lambda_g & lambda_y: coefficient in regulation for nonlinearty and uncertainty
#   u_limit & s_limit:   bound on control input & spacing (optional)
# Output:
#   u_opt:               designed optimal future control input
#   y_opt:               predicted output in the optimal future control input
#   problem_status:      problem status in optimization calculation
#
#                      Optimization Formulation
# mininize:
#   ||y||_{Q_blk}^2 + ||u||_{R_blk}^2 + lambda_g||g||_2^2 + lambda_y||sigma_y||_2^2
# subject to:
#   [Up]    [uini]   [   0   ]
#   [Ep]    [eini]   [   0   ]
#   [Up]g = [yini] + [sigma_y], e = 0, u in u_limit, [0 I_m]y in s_limit
#   [Uf]    [ u  ]   [   0   ]
#   [Ef]    [ e  ]   [   0   ]
#   [Yf]    [ y  ]   [   0   ]
#
# We transform the problem into **standard quadratic programming** for calculation
#
# See Section IV of the following paper for modeling details
#   Title : Data-Driven Predicted Control for Connected and Autonomous
#           Vehicles in Mixed Traffic
#   Author: Jiawei Wang, Yang Zheng, Qing Xu and Keqiang Li
# =========================================================================
def qp_DeeP_LCC(*args):
# def qp_DeeP_LCC(Up,Yp,Uf,Yf,Ep,Ef, uini,yini,eini,Q,R,r,lambda_g,lambda_y,u_limit,s_limit):
    # parsing in all arguments
    Up = args[0];
    Yp = args[1];
    Uf = args[2];
    Yf = args[3];
    Ep = args[4];
    Ef = args[5];
    uini = args[6];
    yini = args[7];
    eini = args[8];
    Q = args[9];
    R = args[10];
    r = args[11];
    lambda_g = args[12];
    lambda_y = args[13];
    constraint_bool = 0;                       # whether there exists input/output constraints
    if len(args) > 14:
        u_limit = args[14];
        s_limit = args[15];
        constraint_bool = 1;
    
    # ------------
    # parameters
    # ------------
    m        = uini.shape[0];                # dimension of control input
    p        = yini.shape[0];                # dimension of output
    Tini     = Up.shape[0] // m;             # horizon of past data
    N        = Uf.shape[0] // m;             # horizon of future data
    T        = Up.shape[1] + Tini + N - 1;   # time length of pre-collected data

    # reshape past data into one single trajectory
    # uini = col(u(-Tini),u(-Tini+1),...,u(-1)) (similarly for yini and eini)
    uini_col = np.reshape(uini, (m * Tini, 1), order="F");
    yini_col = np.reshape(yini, (p * Tini, 1), order="F");
    eini_col = np.reshape(eini, (Tini, 1), order="F");
    r_col    = np.reshape(r, (p * N, 1), order="F");

    Q_blk = np.zeros((p * N, p * N));
    R_blk = np.zeros((m * N, m * N));

    for i in range(1, N+1):
        Q_blk[((i-1)*p+1)-1 : i*p, ((i-1)*p+1)-1 : i*p] = Q;
        R_blk[((i-1)*m+1)-1 : i*m, ((i-1)*m+1)-1 : i*m] = R; 


    # ---------------------
    # cvxopt  - QP solver in Python
    # from cvxopt import solvers
    # sol = solvers.qp(P, q, G, h, A, b) - elements in input matrices are double
    # minimize     0.5*x'*P*x+q'*x    
    # subject to         G*x          <= h 
    #                    A*x           = b
    # ---------------------

    # Coefficient
    P = np.dot(np.dot(Yf.transpose(), Q_blk), Yf);
    P = P + np.dot(np.dot(Uf.transpose(), R_blk), Uf);
    P = P + np.eye(T-Tini-N+1) * lambda_g[0, 0];
    P = P + np.dot(lambda_y[0, 0] * Yp.transpose(), Yp);
    q = np.dot(-1 * lambda_y[0, 0] * Yp.transpose(), yini_col);

    A =  np.vstack((Up, Ep, Ef));
    b =  np.vstack((uini_col, eini_col, np.zeros((N, 1))));

    if constraint_bool: # there exists input/output constraints
        Sf = np.hstack((np.zeros((m, p - m)), np.eye(m)));
        Sf_blk = Sf;
        for i in range(2, N+1):
            Sf_blk = np.block([[Sf_blk, np.zeros((Sf_blk.shape[0], Sf.shape[1]))], [np.zeros((Sf.shape[0], Sf_blk.shape[1])), Sf]]);
        G = np.vstack((Uf, -Uf, np.dot(Sf_blk, Yf), np.dot(-Sf_blk, Yf)));
        h = np.vstack((np.dot(np.max(u_limit), np.ones((m * N, 1))), np.dot(-1 * np.min(u_limit),np.ones((m * N, 1))), np.dot(np.max(s_limit), np.ones((m * N, 1))), np.dot(-1 * np.min(s_limit), np.ones((m * N,1)))));
    else: 
        G = [];
        h = [];

    P = matrix(P, tc = 'd');
    q = matrix(q, tc = 'd');
    G = matrix(G, tc = 'd');
    h = matrix(h, tc = 'd');
    A = matrix(A, tc = 'd');
    b = matrix(b, tc = 'd');
    
    sol = solvers.qp(P, q, G, h, A, b);

    u_opt = np.dot(Uf, sol['x']);
    y_opt = np.dot(Yf, sol['x']);
    exitflag = sol['status'];

    # exitflags:
    # optimal: The algorithm converged to a solution.
    # primal infeasible: The problem is infeasible.
    # dual infeasible: The problem is unbounded.
    # maxiters exceeded: Maximum number of iterations exceeded.
    if exitflag == 'optimal':
        problem_status = 1;
    elif exitflag == 'maxiters exceeded':
        problem_status = 0;
    elif exitflag == 'primal infeasible':
        problem_status = -2;
    else: # exitflag == 'dual infeasible'
        problem_status = -3;

    return u_opt,y_opt,problem_status;