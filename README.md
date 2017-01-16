# pyhiro
python scripts to control the hiro/nxo/hrp5p

The following libraries are used

1) Panda3D
2) Networkx
3) Numpy, Scipy
4) MySQLdb

The following executable programs are used:
1) MySQL set (including server, bench, dump, etc.)
Import 20170113freegrip.sql as into the DBMS system first.

The program follows these common sense:
(1) A 3d point or nd vector is represented using one row of np.array list
(2) A n-by-3 matrix is represented by n rows of np.array list

Executing the program for HRP5P
1) Compute the grasps using freegrip.py (under manipulation/grip)
2) Compute the stable placements using freetabletopplacement.py (under manipulation/grip)
3) Compute the stable placements and ik-feasible grasps all over a table surface using tableplacements.py (under manipulation/regrasp)
4) Build the regrasp graph using regriptpp.py (under manipulation/regrasp)
Step 4) is integrated in regrasp/hrp5plot.py, see the final results by executing this file.