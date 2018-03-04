import numpy as np
import matplotlib.pyplot as plt
import math
import numpy.matlib
from scipy.optimize import nnls
import scipy.io as sio


# data was sampled in 100hz
Pts_size = 100
fs = 100

data = np.loadtxt("./datasets/data.dat")
data = data[:,2:5]*3600
[N, M] = data.shape

n = np.arange(0, math.floor(math.log(N/2, 2))+1)
n = 2**n
maxN = n[n.size-1]
endLogInc = math.log(maxN, 10);
m = np.unique(np.ceil(np.logspace(0, endLogInc, Pts_size))).transpose()

t0 = 1.0/fs
T = m*t0

theta = np.zeros([N, M])
theta[:, 0] = data[:, 0].cumsum()/fs
theta[:, 1] = data[:, 1].cumsum()/fs
theta[:, 2] = data[:, 2].cumsum()/fs
sigma2 = np.zeros([T.size, M])
for i in range(0, m.size):
    for k in range(0, int(N-2*m[i])):
        sigma2[i,:] = sigma2[i,:] + (theta[int(k+2*m[i]), :] - 2*theta[int(k+m[i]), :] + theta[k, :])**2
    print(i)
    sigma2[i,:] = sigma2[i,:]/(2*T[i]**2*(N-2*m[i]))

sigma2 = np.load("sigma2.dat.npy")
T = np.load("T.dat.npy")


sigma = np.sqrt(sigma2)
print("hello")

plt.loglog(T, sigma2)
plt.show()

for j in range(0,2):
    avar = sigma(j)
    P = np.empty((T.size, 5))
    P[:, 0] = 3 / T**2
    P[:, 1] = 1 / T
    P[:, 2] = 2 * np.log(2) / np.pi
    P[:, 3] = T / 3
    P[:, 4] = T**2 / 2
    P /= avar[:, np.newaxis]
    b = np.ones(T.size)
    s = nnls(P, b)[0]
    params = np.sqrt(s)

print(params)