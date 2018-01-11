# pyhiro
python scripts to control the hiro/nxo/hrp5p

The following libraries are used

1. Panda3D
2. Networkx
3. Numpy, Scipy
4. MySQLdb

The following executable programs are used:

1. MySQL set (including server, bench, dump, etc.)
    * Import 20170113freegrip.sql as into the DBMS system first.

The program follows these common sense:

1. A 3d point or nd vector is represented using one row of np.array list
2. A n-by-3 matrix is represented by n rows of np.array list

Executing the program for HRP5P

1. Compute the grasps using freethreegrip.py (under manipulation/grip)
2. Compute the stable placements using freetabletopplacement.py (under manipulation/grip)
3. Compute the stable placements and ik-feasible grasps all over a table surface using tableplacements.py (under manipulation/regrasp)
4. Build the regrasp graph and plan motion sequences hrp5nmixplot_withcd.py (under manipulation/regrasp)

Executing the program for Dual-arm assembly

1. Compute the grasps using freegrip.py (under manipulation/grip)
2. Compute the stable placements using freetabletopplacement.py (under manipulation/grip)
3. Compute the stable placements and ik-feasible grasps all over a table surface using tableplacements.py (under manipulation/regrasp)
4. Execute manipulation/assembly/asstwoobj.py. The script saves/loads assembly, generates assemblyx,
assemblyxgrippairs, assemblyxgripsp0, and assemblyxgrips1. It also updates the ikassemblygrips0 and
ikassemblygrips1. Assembly (relative poses of two objects), Assemblyx: Assembly with rotation.
5. Execute nxttppassplot.py to find one plan

Delete the grasp data (in MySQL workbench):
```sql
SET FOREIGN_KEY_CHECKS=0;
truncate table freegrip.tabletopplacements;
truncate table freegrip.tabletopgrips;
truncate table freegrip.ik;
truncate table freegrip.freeairgrip;
truncate table freegrip.freetabletopplacement;
truncate table freegrip.freetabletopgrip;
SET FOREIGN_KEY_CHECKS=1;
```

Delete the tabletop regrasp data (in MySQL workbench):
```sql
SET FOREIGN_KEY_CHECKS=0;
truncate table tabletopplacements;
truncate table tabletopgrips;
truncate table ik;
SET FOREIGN_KEY_CHECKS=1;
```

Delete the floating pose data (in MySQL workbench):
```sql
SET FOREIGN_KEY_CHECKS=0;
truncate table floatingposes;
truncate table floatinggrips;
truncate table floatinggripspairs;
truncate table ikfloatinggrips;
SET FOREIGN_KEY_CHECKS=1;
```

Delete the dual-arm assembly data (in MySQL workbench):
```sql
SET FOREIGN_KEY_CHECKS=0;
truncate table assembly;
truncate table assemblyx;
truncate table assemblyxgrippairs;
truncate table assemblyxgrips0;
truncate table assemblyxgrips1;
truncate table ikassemblyxgrips0;
truncate table ikassemblyxgrips1;
SET FOREIGN_KEY_CHECKS=1;
```