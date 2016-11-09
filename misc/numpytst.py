import numpy as np

a = np.array([1,2,3])
print a.reshape(-1,1)

import math

print 0.261799*180/math.pi

import utils.robotmath as rm
print rm.rodmat(np.array([0,0,1]), 90)
print rm.rodmat(np.array([0,0,1]), 30)
print rm.rodmat(np.array([0,0,1]), 0)
print rm.rodmat(np.array([0,0,1]), 0)
print rm.rodmat(np.array([0,1,0]), 0)