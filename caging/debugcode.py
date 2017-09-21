#!/usr/bin/python

from matplotlib import pyplot
from shapely.geometry.polygon import LineString, Polygon
from shapely.geometry.point import Point
import numpy as np


polygoncoords = np.array([(292.0302545756387, 1742.179060145323),
                           (-207.9697454243618, 876.1536563608848),
                           (158.0556583600762, -489.871747423554),
                           (1091.068360252296, 126.1536563608842),
                           (1158.055658360077, 1242.179060145323),
                           (292.0302545756387, 1742.179060145323)])
poly = Polygon(polygoncoords.tolist())
fig = pyplot.figure()
ax = fig.add_subplot(111)
x,y = poly.exterior.xy
ax.plot(x, y, color='#6699cc', alpha=0.7,
    linewidth=3, solid_capstyle='round', zorder=2)
ax.set_title('Polygon')
ax.axis('equal')
ax.axis([-1000, 2000, -1000, 2000])

ax.plot([731.8212389159114,0], [765.4088120040248,0], color='#ff0000', alpha=0.7,
    linewidth=10, solid_capstyle='round', zorder=2)
pyplot.show()

# grp0 = np.array([])
# grp1 = np.array([])
# vec = grp1-grp0
# veclength = np.linalg.norm(vec)
# vecdirect = vec/veclength
#
# grppoint0 = Point(-700.0, 1420.0)
# grppoint1 = Point(1440.0, 820.0)
#
# vecstep = veclength/10.0
# for i in range(0,5):
#     print [vecstep*i*vecdirect/2.0+grp0, -vecstep*i*vecdirect/2.0+grp1]