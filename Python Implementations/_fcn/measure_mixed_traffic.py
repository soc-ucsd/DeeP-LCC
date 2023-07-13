import numpy as np

# =========================================================================
#               Measure the output in mixed traffic flow
#
# vel:      velocity of each vehicle
# pos:      position of each vehicle
# ID:       ID of vehicle types     1: CAV  0: HDV
# v_star:   equilibrium velocity
# s_star:   equilibrium spacing
# type:
# 1. Only the velocity errors of all the vehicles are measurable;
# 2. All the states, including velocity error and spacing error are measurable;
# 3. Velocity error and spacing error of the CAVs are measurable, 
#    and the velocity error of the HDVs are measurable.
# =========================================================================

def measure_mixed_traffic (vel,pos,ID,v_star,s_star,type) :
    # pos_cav = np.where(ID == 1)[1]
    pos_cav = np.argwhere(np.ravel(ID) == 1)

    if type == 1:
            y= np.transpose([vel - v_star])
    elif type == 2:
            spacing = pos[0:-1] - pos[1:]
            y = np.vstack((np.transpose([vel - v_star]), np.transpose([spacing - s_star])))
    elif type == 3:
            spacing = pos[0:-1] - pos[1:]
            y = np.vstack(((vel - v_star).reshape((8,1)), spacing[pos_cav] - s_star))
    return y
    

# Testing
# vel1 =  np.matrix('14.9954   14.9985   14.9889   14.9970   14.9922   14.9696   14.9880   14.9787');
# pos1 = np.matrix('513.5000  492.0058  473.9977  454.0467  435.0456  414.0380  394.1034  372.1208  352.6225');
# ID1 = np.matrix('0     0     1     0     0     1     0     0');
# v_star1 = 15.0000;
# s_star1 = 20.0000;
# type1 = 2;
# measure_mixed_traffic(vel1,pos1,ID1,v_star1,s_star1,type1)