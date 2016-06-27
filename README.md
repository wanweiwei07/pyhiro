# pyhiro
python scripts to control the hiro/nxo

The following libraries are used

1) trimesh for mesh processing

https://pypi.python.org/pypi/trimesh

The program follows these common sense:
(1) A 3d point or nd vector is represented using one row of np.array list
(2) A n-by-3 matrix is represented by n rows of np.array list

TODO:
(1) when creating a panda geom from trimesh, the points are added following
faces. They are repeated.
(2) nodepath setcolor doesnt seem to affect its children (representto@star)