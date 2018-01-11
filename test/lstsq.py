import numpy as np
import transformations as tr

file = open('pts_c.txt', 'r')
pts_c = []
for line in file:
    lineele = line.split()
    pts_c.append([float(lineele[0]), float(lineele[1]), float(lineele[2])])
print pts_c

file2 = open('pts_w.txt', 'r')
pts_w = []
for line in file2:
    lineele = line.split()
    pts_w.append([float(lineele[0]), float(lineele[1]), float(lineele[2])])
print pts_w

pts_c_rel = []
pts_w_rel = []
for i in range(len(pts_c)-1):
    pts_c_rel.append([pts_c[i][0]-pts_c[len(pts_c)-1][0], pts_c[i][1]-pts_c[len(pts_c)-1][1], pts_c[i][2]-pts_c[len(pts_c)-1][2]])
    pts_w_rel.append([pts_w[i][0]-pts_w[len(pts_c)-1][0], pts_w[i][1]-pts_w[len(pts_c)-1][1], pts_w[i][2]-pts_w[len(pts_c)-1][2]])
print pts_c_rel
print pts_w_rel

pts_c_rel_np = np.array(pts_c_rel)
pts_w_rel_np = np.array(pts_w_rel)
M = tr.affine_matrix_from_points(pts_c_rel_np.T, pts_w_rel_np.T, shear = False)

const = -np.dot(M, np.array([pts_c[len(pts_c)-1][0], pts_c[len(pts_c)-1][1], pts_c[len(pts_c)-1][2], 1]))+\
        np.array([pts_w[len(pts_c)-1][0], pts_w[len(pts_c)-1][1], pts_w[len(pts_c)-1][2], 0])
print const
print np.dot(M, np.array([pts_c[0][0], pts_c[0][1], pts_c[0][2], 1]))+const