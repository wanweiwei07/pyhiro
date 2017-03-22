from panda3d.core import *

vecx = Vec3(0,-137.249,-77.004)
xmat = Mat4.rotateMat(-45, Vec3(1,0,0))
print xmat.xformVec(vecx)