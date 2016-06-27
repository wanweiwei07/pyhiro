from shapely.geometry import Polygon
import matplotlib.pyplot as plt

ext = [(0, 0), (0, 1), (1, 1), (1, 0), (0, 0)]
int = [(0.5, 0), (0.25, 0.25), (0.5, 0.5), (0.75, 0.25), (0.5, 0)][::-1]
p1 = Polygon(ext, [int])
p2 = Polygon([(0,1),(1,1),(2,1),(2,2)])
newp = p1.union(p2)

p1x, p1y = p1.exterior.xy
p1xi, p1yi = p1.interiors[0].xy
p2x, p2y = p2.exterior.xy
unionx, uniony = newp.exterior.xy
unionxi, unionyi = newp.interiors[0].xy
# plt.plot(p1x, p1y)
# plt.plot(p1xi, p1yi)
# plt.plot(p2x, p2y)
plt.plot(unionx, uniony)
plt.plot(unionxi, unionyi)
plt.show()