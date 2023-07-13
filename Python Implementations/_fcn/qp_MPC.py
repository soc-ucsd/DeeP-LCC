from quadprog import *
import scipy.linalg
import cvxopt
from cvxopt import solvers
from traffic_linear_model import traffic_linear_model

def qp_MPC(*args):
    '''
            MPC for mixed traffic
    Input:
      ID:                 vehicle ID
      Ts:                 sampling time
      hdv_type:           type of HDV car-following model
      measure_type:       measure type for output definition
      v_star:             equilibrium velocity
      uini & yini:        past data of length Tini in control process
      N:                  future time length (predicted horizon)
      Q & R:              weight coefficient matrix in performance cost
      r:                  reference trajectory
      u_limit & s_limit:  bound on control input & spacing
      previous_u_opt:     control input in previous time step
    Output:
      u_opt:               designed optimal future control input
      y_opt:               predicted output in the optimal future control input
      problem_status:      problem status in optimization calculation

                        Optimization Formulation
    mininize:
      ||y||_{Q_blk}^2 + ||u||_{R_blk}^2
    subject to:
      xini is estimated from past data uini,yini 
      x    = Ax + Bu
      y    = Cx

    We transform the problem into **standard quadratic programming** for calculation

    See Section IV of the following paper for modeling details
      Title : Data-Driven Predicted Control for Connected and Autonomous
              Vehicles in Mixed Traffic
      Author: Jiawei Wang, Yang Zheng, Qing Xu and Keqiang Li
    '''

    ID = args[0]
    Ts = args[1]
    hdv_type = args[2]
    measure_type = args[3]
    v_star = args[4]
    uini = args[5]
    yini = args[6]
    N = args[7]
    Q = args[8]
    R = args[9]
    r = args[10]
    u_limit = args[11]
    s_limit = args[12]
    if len(args) > 13:
        previous_u_opt = args[13]

    # Calculate via Linear model
    if len(args) < 12:       # whether there exists input/output constraints
        constraint_bool = 0
    else:
        constraint_bool = 1

    # linear model
    [A, B, C] = traffic_linear_model(ID, Ts, hdv_type, measure_type, v_star)
    # dimension
    m         = len(uini)    # dimension of control input
    p         = len(yini)    # dimension of output
    n         = len(A)       # dimension of state
    Tini      = len(uini[0]) # horizon of past data

    # reshape past data into one single trajectory
    uini_col  = np.reshape(uini, (m * Tini,1), order="F")
    yini_col  = np.reshape(yini, (p * Tini,1), order="F")

    Obsv_Tini = np.zeros((p * Tini, n))
    for i in range(0, Tini):
        Obsv_Tini[i * p: (i + 1) * p, :] = C @ A ** i

    Toep_Tini = np.zeros((p * Tini, m * Tini))
    Toep_Tini[1 * p: 2 * p, 0 * m: 1 * m] = C @ B
    for i in range(3, Tini+1):
        for j in range(i-1):
            Toep_Tini[(i - 1) * p: i * p, j * m: (j + 1) * m] = C @ A ** (i - j) @ B

    Obsv_N = np.zeros((p * N, n))
    for i in range(N):
        Obsv_N[i * p: (i + 1) * p, :] = C @ A ** i

    Toep_N = np.zeros((p * N, m * N))
    Toep_N[1 * p: 2 * p, 0 * m: 1 * m] = C @ B
    for i in range(3, N+1):
        for j in range(i-1):
            Toep_N[(i - 1) * p: i * p, j * m: (j + 1) * m] = C @ A ** (i - j) @ B

    Ctrb_Tini = np.zeros((n, m * Tini))
    for i in range(0, Tini):
        Ctrb_Tini[:, i * m: (i + 1) * m] = A ** (Tini - i - 1) @ B

    x_1 = np.linalg.pinv(Obsv_Tini) @ (yini_col - Toep_Tini @ uini_col)
    x_Tini_plus1 = A ** Tini @ x_1 + Ctrb_Tini @ uini_col

    r_col = np.reshape(r, (p * N, 1), order="F")

    Q_blk = np.zeros((p * N, p * N))
    R_blk = np.zeros((m * N, m * N))

    for i in range(0, N):
        Q_blk[i * p: (i + 1) * p, i * p: (i + 1) * p] = Q
        R_blk[i * m: (i + 1) * m, i * m: (i + 1) * m] = R

    Cu = Obsv_N @ (Ctrb_Tini - A ** Tini @ np.linalg.pinv(Obsv_Tini) @ Toep_Tini) @ uini_col + Obsv_N @ A ** Tini @ np.linalg.pinv(Obsv_Tini) @ yini_col

    '''
    Standard QP in MATLAB
    [x,fval,exitflag,output,lambda]=quadprog(H,f,A,b,B,c,l,u,x0,options)
    minimize     0.5*x'*H*x+f'*x    
    subject to        A*x          <= b 
                      B*x           = c
                      l <= x <= u
    '''

    # Coefficient
    H = Toep_N.T @ Q_blk @ Toep_N + R_blk
    f = Toep_N.T @ Q_blk @ Obsv_N @ x_Tini_plus1

    if constraint_bool:   # there exists input/output constraints
        Sf = np.hstack((np.zeros((m, p - m)), np.eye(m)))
        Sf_blk = Sf
        for i in range(1, N):
            Sf_blk = scipy.linalg.block_diag(Sf_blk, Sf)
        A = np.vstack((np.eye(m * N), - np.eye(m * N), Sf_blk @ Toep_N, - Sf_blk @ Toep_N))
        b = np.vstack((np.max(u_limit) * np.ones((m * N, 1)), - np.min(u_limit) * np.ones((m * N, 1)), np.max(s_limit) * np.ones((m * N, 1)) - Sf_blk @ Cu, - np.min(s_limit) * np.ones((m * N, 1)) + Sf_blk @ Cu))
    else:
        A = []
        b = []

    # Optimization
    H = cvxopt.matrix(H)
    f = cvxopt.matrix(f)
    A = cvxopt.matrix(A)
    b = cvxopt.matrix(b)

    sol = solvers.qp(H, f, A, b)
    u_opt = sol['x']
    exitflag = sol['status']

    if sol['status'] == 'optimal':
        exitflag = 1
    elif sol['status'] == 'primal infeasible' or sol['status'] == 'dual infeasible':
        exitflag = -2
        u_opt = previous_u_opt
    elif sol['status'] == 'unknown':
        exitflag = 0
        u_opt = previous_u_opt

    # Solution
    y_opt = Obsv_N @ x_Tini_plus1 + Toep_N @ u_opt
    problem_status = exitflag

    return u_opt, y_opt, problem_status
