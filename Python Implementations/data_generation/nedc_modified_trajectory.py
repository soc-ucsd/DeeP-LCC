import numpy as np
import matplotlib.pyplot as plt

Tstep = 0.05
total_time = 10 + 8 + 20 + 13 + 20 + 25 + 20 + 10 + 20
time = np.arange(0, total_time, Tstep)

vel = np.zeros(len(time))

cruise_time_i = 1
change_time_i = 1
for i in range(len(time)):
    if time[i] <= 10:
        vel[i] = 70
    elif time[i] <= 10 + 8:
        vel[i] = 70 - 20 / 8 * (time[i] - 10)
    elif time[i] <= 10 + 8 + 20:
        vel[i] = 50
    elif time[i] <= 10 + 8 + 20 + 13:
        vel[i] = 50 + 20 / 13 * (time[i] - (10 + 8 + 20))
    elif time[i] <= 10 + 8 + 20 + 13 + 20:
        vel[i] = 70
    elif time[i] <= 10 + 8 + 20 + 13 + 20 + 25:
        vel[i] = 70 + 30 / 25 * (time[i] - (10 + 8 + 20 + 13 + 20))
    elif time[i] <= 10 + 8 + 20 + 13 + 20 + 25 + 20:
        vel[i] = 100
    elif time[i] <= 10 + 8 + 20 + 13 + 20 + 25 + 20 + 10:
        vel[i] = 100 - 30 / 10 * (time[i] - (10 + 8 + 20 + 13 + 20 + 25 + 20))
    elif time[i] <= 10 + 8 + 20 + 13 + 20 + 25 + 20 + 10 + 20:
        vel[i] = 70

plt.plot(time, vel)
plt.show()
