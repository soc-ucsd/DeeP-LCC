import numpy as np

def hankel_matrix(u,L):
    '''

    Generate a Hankel matrix of order L

    '''
    
    m = u.shape[0]
    T = u.shape[1]
    U = np.zeros((m*L, T-L+1))

    for i in range(1, L+1):
        U[(i-1)*m:i*m, :] = u[:, i-1:T-L+i]
        
    return U
