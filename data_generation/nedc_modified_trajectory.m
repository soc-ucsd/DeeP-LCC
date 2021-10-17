clc;
clear;
close all;

Tstep          = 0.05;
total_time     = 10+8+20+13+20+25+20+10+20;
time           = 1:Tstep:total_time;

vel = zeros(total_time/Tstep,1);

cruise_time_i = 1;
change_time_i = 1;
for i = 1:total_time/Tstep
   if i*Tstep <= 10
       vel(i) = 70;
   elseif i*Tstep <= 10+8
       vel(i) = 70 - 20/8*(i*Tstep-10);
   elseif i*Tstep <= 10+8+20
       vel(i) = 50;
   elseif i*Tstep <= 10+8+20+13
       vel(i) = 50 + 20/13*(i*Tstep-(10+8+20));
   elseif i*Tstep <= 10+8+20+13+20
       vel(i) = 70;
   elseif i*Tstep <= 10+8+20+13+20+25
       vel(i) = 70 + 30/25*(i*Tstep-(10+8+20+13+20));
   elseif i*Tstep <= 10+8+20+13+20+25+20
       vel(i) = 100; 
   elseif i*Tstep <= 10+8+20+13+20+25+20+10
       vel(i) = 100 - 30/10*(i*Tstep-(10+8+20+13+20+25+20));
   elseif i*Tstep <= 10+8+20+13+20+25+20+10+20
       vel(i) = 70;
   end
end
plot(vel);
% save('.\_data\nedc_modified_v2.mat','time','vel');