import numpy as np

a = [1,2,
     3,4]
b = np.asarray(a)
print b.reshape((-1,1))
print b