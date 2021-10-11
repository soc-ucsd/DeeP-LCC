clc;
clear;
close all;

Tstep          = 0.05;
total_time     = 50+8+69+13+50+35+30+20+10+14+20;
time           = 1:Tstep:total_time;

vel = zeros(total_time/Tstep,1);

cruise_time_i = 1;
change_time_i = 1;
for i = 1:total_time/Tstep
   if i*Tstep <= 50
       vel(i) = 70;
   elseif i*Tstep <= 50+8
       vel(i) = 70 - 20/8*(i*Tstep-50);
   elseif i*Tstep <= 50+8+69
       vel(i) = 50;
   elseif i*Tstep <= 50+8+69+13
       vel(i) = 50 + 20/13*(i*Tstep-(50+8+69));
   elseif i*Tstep <= 50+8+69+13+50
       vel(i) = 70;
   elseif i*Tstep <= 50+8+69+13+50+35
       vel(i) = 70 + 30/35*(i*Tstep-(50+8+69+13+50));
   elseif i*Tstep <= 50+8+69+13+50+35+30
       vel(i) = 100; 
   elseif i*Tstep <= 50+8+69+13+50+35+30+20
       vel(i) = 100 + 20/20*(i*Tstep-(50+8+69+13+50+35+30));
   elseif i*Tstep <= 50+8+69+13+50+35+30+20+10
       vel(i) = 120; 
   elseif i*Tstep <= 50+8+69+13+50+35+30+20+10+14
       vel(i) = 120 - 50/14*(i*Tstep-(50+8+69+13+50+35+30+20+10));
   elseif i*Tstep <= 50+8+69+13+50+35+30+20+10+14+20
       vel(i) = 70;
   end
end
plot(vel);

save('nedc.mat','time','vel');