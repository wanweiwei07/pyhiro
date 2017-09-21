#!/usr/bin/python

from matplotlib import pyplot
from shapely.geometry.polygon import LineString, Polygon
from shapely.geometry.point import Point
import numpy as np

# for linearsqueezing fingers
grp0 = np.array([-700.0, 1220.0])
grp1 = np.array([1440.0, 620.0])
vec = grp1-grp0
veclength = np.linalg.norm(vec)
vecdirect = vec/veclength

vecstep = veclength/10.0
for i in range(0,5):
    print [vecstep*i*vecdirect/2.0+grp0, -vecstep*i*vecdirect/2.0+grp1]