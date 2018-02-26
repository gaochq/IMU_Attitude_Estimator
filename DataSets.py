import numpy as np
import scipy.io as sio

load_fn = './datasets/NAV.mat'
load_Data = sio.loadmat(load_fn)
data = load_Data['NAV']

mea = data[:,8:17]
euler = data[:,29:32]

out = np.hstack((mea, euler))

np.savetxt("data.bin", out, fmt='%f')

print out.shape

print data