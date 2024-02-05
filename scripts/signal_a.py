import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
import h5py
import time

A = h5py.File("/home/hami/rgbd_wireless_pos_a.mat", "r")
EE_pos = A["EE_position"][()]

T = A["time"][()]
rgbd_pos = A["rgbd_odom_position"][()]
t = T - T.min()
# plt.plot(t, rgbd_pos[:, 1], label='Original Data')
# plt.show()
print(rgbd_pos.shape)

k, l = rgbd_pos.shape

for i in range(k):
    x, y, z = rgbd_pos[i, :]
    Pos = np.array([[x, y, z]])
    if i==0:
        Pos_array = Pos
    else:
        Pos_array = np.concatenate((Pos_array, Pos), axis=0)
    
    if i>200:
        Pos_fil = signal.savgol_filter(Pos_array.T, window_length=169 , polyorder=1,deriv=0, mode="interp")
    else:
        Pos_fil = signal.savgol_filter(Pos_array.T, window_length=169 , polyorder=1, mode="nearest")
    if i>50 and i%5==0:
        plt.scatter(t[i:i+1, :], Pos_fil.T[-2:-1, 1], s=2)
        #plt.show()
        plt.pause(0.0000000000000000000000000005)

        if i>7000:
            plt.plot(t, rgbd_pos[:, 1], label='Original Data')
            plt.show()


    # if i>50:
    #     if i==7000:
    #         plt.plot(t, rgbd_pos[:, 1], label='Original Data')
    #         plt.plot(t[:i+1, :], Pos_fil.T[:, 1])
    #         plt.show()



