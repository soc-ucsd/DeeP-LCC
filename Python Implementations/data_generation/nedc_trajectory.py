import dill
import numpy as np
from scipy.io import savemat

Tstep = 0.05;
total_time = 50+8+69+13+50+35+30+20+10+14+20;
time = np.arange(1, total_time+1, Tstep);

vel = np.zeros(((int)(total_time/Tstep), 1));

cruise_time_i = 1;
change_time_i = 1;
for i in range(0, (int)(total_time/Tstep)):
   if i*Tstep <= 50:
       vel[i] = 70;
   elif i*Tstep <= 50+8:
       vel[i] = 70 - 20/8*(i*Tstep-50);
   elif i*Tstep <= 50+8+69:
       vel[i] = 50;
   elif i*Tstep <= 50+8+69+13:
       vel[i] = 50 + 20/13*(i*Tstep-(50+8+69));
   elif i*Tstep <= 50+8+69+13+50:
       vel[i] = 70;
   elif i*Tstep <= 50+8+69+13+50+35:
       vel[i] = 70 + 30/35*(i*Tstep-(50+8+69+13+50));
   elif i*Tstep <= 50+8+69+13+50+35+30:
       vel[i] = 100; 
   elif i*Tstep <= 50+8+69+13+50+35+30+20:
       vel[i] = 100 + 20/20*(i*Tstep-(50+8+69+13+50+35+30));
   elif i*Tstep <= 50+8+69+13+50+35+30+20+10:
       vel[i] = 120; 
   elif i*Tstep <= 50+8+69+13+50+35+30+20+10+14:
       vel[i] = 120 - 50/14*(i*Tstep-(50+8+69+13+50+35+30+20+10));
   else: # i*Tstep <= 50+8+69+13+50+35+30+20+10+14+20
       vel[i] = 70;

# save('nedc.mat','time','vel');

data_dict = {'time': time, 'vel': vel}

savemat('nedc.mat', data_dict)

#dill.dump_session('nedc.pkl')