import matplotlib.pyplot as plt
import numpy as np

alpha = 0.6
beta = 0.9

s_st = 5
s_go = 35

v_max = 30

v = np.arange(0, v_max + 0.05, 0.05)
s = np.arccos(1 - v / 30 * 2) / np.pi * (35 - 5) + 5

plt.figure(1)
plt.plot(v, s, linewidth=2)
plt.axis([0, 35, 0, 40])

plt.plot(s, v_max * np.ones_like(s), '--k', linewidth=1)

Wsize = 14
plt.gca().tick_params(labelsize=11)
plt.grid(True)
plt.xlabel('Spacing', fontsize=Wsize, color='k')
plt.ylabel('Desired velocity', fontsize=Wsize, color='k')

plt.gca().set_yticklabels([])
plt.gca().set_xticklabels([])

plt.text(1, 32, '$v_{\mathrm{max}}$', fontsize=Wsize, color='k')

plt.text(30, 3, '$s_{\mathrm{go}}$', fontsize=Wsize, color='k')
plt.text(4.5, 3, '$s_{\mathrm{st}}$', fontsize=Wsize, color='k')
plt.plot([5, 5], [-1, 1], '-k', linewidth=1)
plt.plot([35, 35], [0, 30], '--k', linewidth=1)
plt.subplots_adjust(left=0.2, bottom=0.15, right=0.8, top=0.85)

fig = plt.gcf()
fig.savefig('./Figures_Fig2_OVMSpacingPolicy', format='eps', dpi=300)
plt.show()

