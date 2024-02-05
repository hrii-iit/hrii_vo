import numpy as np
import scipy
from scipy import signal
import matplotlib.pyplot as plt
import h5py
import time
from scipy.linalg import block_diag, inv



A = h5py.File("/home/hami/rgbd_wireless_pos_a.mat", "r")
EE_pos = A["EE_position"][()]

T = A["time"][()]
rgbd_pos = A["rgbd_odom_position"][()]
t = T - T.min()
# plt.plot(t, rgbd_pos[:, 1], label='Original Data')
# plt.show()
print(rgbd_pos.shape)

k, l = rgbd_pos.shape

x_O = np.array([0.0, 0.0, 0.0])
P_O = np.diag([0.04, 0.04, 0.04])



A = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])

Q = np.diag([0.002, 0.002, 0.002])
H = np.eye(3)
R = np.diag([2.5, 2.5, 2.5])
x_hat = x_O
P = P_O

for i in range(k):
    x, y, z = rgbd_pos[i, :]
    Pos = np.array([x, y, z])
    Pos = Pos.reshape((3, 1))
    x_hat_minus = np.dot(A, x_hat)
    P_minus = np.dot(np.dot(A, P), A.T) + Q

    K = np.dot(np.dot(P_minus, H.T), inv(np.dot(np.dot(H, P_minus), H.T) + R))
    x_hat = x_hat_minus + np.dot(K, Pos - np.dot(H, x_hat_minus))
    Pos_hat = np.average(x_hat, axis=1)
    Pos_hat = Pos_hat.reshape((1, 3))
    P = P_minus - np.dot(np.dot(K, H), P_minus)
    if i==0:
        Pos_array = Pos_hat
    else:
        Pos_array = np.concatenate((Pos_array, Pos_hat), axis=0)
plt.scatter(t, Pos_array[:, 1], s=1)
plt.show()
    # print(Pos_array.shape)

print(Pos_array.shape)
# print(rgbd_pos.shape)

    