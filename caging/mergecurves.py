#!/usr/bin/python

from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LineString
from stance.stability import Stability as Stb
from stance.lblstability import Lblstability as Lstb
import os
import math
from panda3d.core import *
import pandaplotutils.pandageom as pg
import pandaplotutils.pandactrl as pc
import thread
import matplotlib.pyplot as plt

class Mergecurves(object):

    def __init__(self, rotrange, scale, steplength):
        """

        :param rotrange: 360
        :param scale:
        :param steplength:

        author: weiwei
        date: 20170822
        """

        self.rotrange = rotrange
        self.scale = scale
        self.heightrange = int(self.rotrange*self.scale)
        self.steplength = steplength
        self.stb = Stb(rotrange, scale, steplength)
        self.lstb = Lstb(rotrange, scale, steplength)
        self.minusinf = -9999999999
        self.plusinf = 9999999999
        self.distthreshold = 1.0

    def cvtcurveList2Dict(self, onedcurves):
        onedcurvedict = {}
        for i in range(0, self.heightrange, self.steplength):
            onedcurvedict[i] = [self.minusinf, self.minusinf, i]
        for i, onedcurve in enumerate(onedcurves):
            for j, point in enumerate(onedcurve):
                onedcurvedict[point[2]] = point
        return onedcurvedict

    def cvtcurveDict2List(self, onedcurvedict):
        onedcurves = []
        onedcurve = []
        for i in range(0, self.heightrange, self.steplength):
            if onedcurvedict[i][0] < self.minusinf+1:
                if len(onedcurve) != 0:
                    onedcurves.append(onedcurve)
                    onedcurve = []
            elif i >= self.heightrange - self.steplength:
                onedcurves.append(onedcurve)
                onedcurve = []
            else:
                onedcurve.append(onedcurvedict[i])
        return onedcurves

    def cvtpolygonList2Dict(self, polygonxoncurves, onedcurves):
        polygonxoncurvedict = {}
        for i in range(0, self.heightrange, self.steplength):
            polygonxoncurvedict[i] = None
        for i, polygonxoncurve in enumerate(polygonxoncurves):
            for j, polygonx in enumerate(polygonxoncurve):
                height = onedcurves[i][j][2]
                polygonxoncurvedict[height] = polygonx
        return polygonxoncurvedict

    def cvtpolygonDict2List(self, polygonxoncurvedict):
        polygonxoncurves = []
        polygonxoncurve = []
        for i in range(0, self.heightrange, self.steplength):
            if polygonxoncurvedict[i] is None:
                if len(polygonxoncurve) != 0:
                    polygonxoncurves.append(polygonxoncurve)
                    polygonxoncurve = []
            elif i >= self.heightrange - self.steplength:
                polygonxoncurves.append(polygonxoncurve)
                polygonxoncurve = []
            else:
                polygonxoncurve.append(polygonxoncurvedict[i])
        return polygonxoncurves

    def removeCollisiononOnedCurves(self, onedcurvedict, polygonxoncurvedict, collidingPolygondict):
        """
        remove the nodes that collide with collidinPolygondict
        :param onedcurvedict: the curve for collision test
        :param polygonxoncurvedict: the curvedict to be updated
        :param collidingPolygondict: the polygon for collision detection
        :return: an updated oned curve dict

        author: weiwei
        date: 20170512
        """
        newonedcurvedict = {}
        newpolygonxoncurvedict = {}
        maxkey = max(onedcurvedict.keys())
        minkey = min(onedcurvedict.keys())
        for key, point in onedcurvedict.iteritems():
            newonedcurvedict[key] = [self.minusinf, self.minusinf, key]
            newpolygonxoncurvedict[key] = None
            polygonx = collidingPolygondict[key]
            if not polygonx.contains(Point(point[0], point[1])):
                newonedcurvedict[key] = point
                newpolygonxoncurvedict[key] = polygonxoncurvedict[key]
            elif polygonx.exterior.contains(Point(point[0], point[1])):
                newonedcurvedict[key] = point
                newpolygonxoncurvedict[key] = polygonxoncurvedict[key]
            else:
                # check jumps
                pkey = key - self.steplength
                nkey = key + self.steplength
                if nkey <= maxkey:
                    npoint = onedcurvedict[nkey]
                    if npoint[0] > self.minusinf+1:
                        if not polygonx.contains(Point(npoint[0], npoint[1])):
                            pathseg = LineString([(point[0], point[1]), (npoint[0], npoint[1])])
                            cobsintersections = pathseg.intersection(polygonx.boundary)
                            if cobsintersections.geom_type == "Point":
                                newonedcurvedict[key] = [cobsintersections.x, cobsintersections.y, point[2]]
                                newpolygonxoncurvedict[key] = polygonxoncurvedict[key]
                            continue
                if pkey >= minkey:
                    ppoint = onedcurvedict[pkey]
                    if ppoint[0] > self.minusinf+1:
                        if not polygonx.contains(Point(ppoint[0], ppoint[1])):
                            pathseg = LineString([(point[0], point[1]), (ppoint[0], ppoint[1])])
                            cobsintersections = pathseg.intersection(polygonx.boundary)
                            if cobsintersections.geom_type == "Point":
                                newonedcurvedict[key] = [cobsintersections.x, cobsintersections.y, point[2]]
                                newpolygonxoncurvedict[key] = polygonxoncurvedict[key]
                            continue

        return [newonedcurvedict, newpolygonxoncurvedict]

    def plotCobt(self, nodepath, point, polygon, rgba = [.5, .5, .5, 1]):
        """
        generate the configuration obstacle

        :param point:
        :param polygon:
        :return:

        author: weiwei
        date: 20170517
        """

        # plot
        cobt = self.stb.genMinkovPnt(polygon, point)
        polygonlist = []
        for height in range(0, self.heightrange, self.steplength):
            rotedcobt = self.stb.rotate(cobt, height/self.scale)
            polygonlist.append(rotedcobt)
        cobtnp = self.stb.genCobtnp(polygonlist, self.steplength)
        cobtnp.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
        cobtnp.setTransparency(TransparencyAttrib.MAlpha)
        cobtnp.reparentTo(nodepath)

    def plotCobtSeg(self, nodepath, segment, polygon, rgba = [.5, .5, .5, 1]):
        """
        generate the configuration obstacle
        obstacles are separated

        :param point:
        :param polygon:
        :return:

        author: weiwei
        date: 20170924, Vancouver
        """

        # plot
        polygonlist = []
        for height in range(0, self.heightrange, self.steplength):
            rotedcobt = self.stb.genMinkovSeg(self.stb.rotate(polygon, height/self.scale), segment)
            polygonlist.append(rotedcobt)
        cobtnp = self.stb.genCobtnp(polygonlist, self.steplength)
        cobtnp.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
        cobtnp.setTransparency(TransparencyAttrib.MAlpha)
        cobtnp.reparentTo(nodepath)

    def plotCobtDis(self, nodepath, point, polygon, rgba = [.5, .5, .5, 1]):
        """
        generate the configuration obstacle

        :param point:
        :param polygon:
        :return:

        author: weiwei
        date: 20170517
        """

        # plot
        cobt = self.stb.genMinkovPnt(polygon, point)
        polygonlist = []
        for height in range(0, self.heightrange, self.steplength):
            rotedcobt = self.stb.rotate(cobt, height/self.scale)
            polygonlist.append(rotedcobt)
        cobtnp, polynps = self.stb.genCobtDisnp(polygonlist, self.steplength)
        cobtnp.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
        cobtnp.setTransparency(TransparencyAttrib.MAlpha)
        cobtnp.reparentTo(nodepath)
        for polynp in polynps:
            polynp.setColor(0,0,0,1)
            polynp.reparentTo(nodepath)

    def plot2d(self, polygon, grppoint0, grppoint1, sppoint0, sppoint1, polygonfacecolor='g'):
        fig = plt.figure()
        ax2 = fig.add_subplot(111)
        xpolygon, ypolygon = polygon.exterior.xy
        ax2.fill(xpolygon, ypolygon, alpha=1, fc=polygonfacecolor, ec='none')
        ax2.plot([grppoint0.x], [grppoint0.y], 'ro')
        ax2.plot([grppoint1.x], [grppoint1.y], 'ro')
        ax2.plot([sppoint0.x], [sppoint0.y], 'ko')
        ax2.plot([sppoint1.x], [sppoint1.y], 'ko')
        ax2.plot([polygon.centroid.x], [polygon.centroid.y], 'bo')
        ax2.axis('equal')
        ax2.axis([-500, 2500, -500, 2500])

        plt.show()
        pass

    def plot2d_givenax(self, ax, polygonlist, polygonfacecolorlist, pointlist=[]):
        print pointlist
        for i, point in enumerate(pointlist):
            point_p = Point(point[0], point[1])
            xpoint, ypoint = point_p.xy
            ax.plot(xpoint, ypoint, 'ro', linewidth=3)
        for i, polygon in enumerate(polygonlist):
            polygonfacecolor = polygonfacecolorlist[i]
            xpolygon, ypolygon = polygon.exterior.xy
            ax.fill(xpolygon, ypolygon, alpha=.3, fc=polygonfacecolor, ec='none')

    def plot2d_list(self, polygonlist, grppoint0, grppoint1, sppoint0, sppoint1, polygonfacecolorlist):
        fig = plt.figure()
        ax2 = fig.add_subplot(111)
        for i, polygon in enumerate(polygonlist):
            polygonfacecolor = polygonfacecolorlist[i]
            xpolygon, ypolygon = polygon.exterior.xy
            ax2.fill(xpolygon, ypolygon, alpha=.3, fc=polygonfacecolor, ec='none')
            if i == 1:
                ax2.plot([polygon.centroid.x], [polygon.centroid.y], 'go')
            else:
                ax2.plot([polygon.centroid.x], [polygon.centroid.y], 'bo')
            ax2.axis('equal')
            ax2.axis([-3000, 4000, -1200, 3000])
        polyunion = polygonlist[-1]
        for i in range(-4,-1,1):
            polyunion = polyunion.union(polygonlist[i])
        if polyunion.geom_type == 'MultiPolygon':
            mindist = self.plusinf
            resultingpoly = None
            for poly in polyunion:
                x,y = poly.exterior.xy
                ax2.plot(x, y, color='#888888', alpha=0.7,
                    linewidth=3, solid_capstyle='round', zorder=2)
                if poly.contains(polygonlist[1].centroid) or polygonlist[1].centroid.distance(poly) < self.distthreshold:
                    resultingpoly = poly
                    break
            polyunion = resultingpoly
        x, y = polyunion.exterior.xy
        ax2.plot(x, y, color='#ff0000', alpha=0.7,
            linewidth=3, solid_capstyle='round', zorder=2)

        ax2.plot([polygonlist[1].centroid.x], [polygonlist[1].centroid.y], 'yo')
        ax2.plot([polygonlist[0].centroid.x], [polygonlist[0].centroid.y], 'ro')
        ax2.plot([grppoint0.x], [grppoint0.y], 'go')
        ax2.plot([grppoint1.x], [grppoint1.y], 'go')
        ax2.plot([sppoint0.x], [sppoint0.y], 'ko')
        ax2.plot([sppoint1.x], [sppoint1.y], 'ko')

        curvepoint_shapely = polygonlist[1].centroid
        # ax2.plot([curvepoint_shapely.x], [curvepoint_shapely.y], 'ro', markersize = 30)
        boundary_shapely = polyunion.exterior
        # print len(list(polyunion.interiors))
        for bd in polyunion.interiors:
            x, y = bd.xy
            ax2.plot(x, y, color='#fff000', alpha=0.7,
                linewidth=3, solid_capstyle='round', zorder=2)
            # if bd.distance(curvepoint_shapely)

        # maxima_xplus = self.lstb.boundarymaxima(curvepoint_shapely, boundary_shapely, [], "idxplus")
        # ax2.plot([maxima_xplus.x], [maxima_xplus.y], 'yo', markersize =10)
        # maxima_xminus = self.lstb.boundarymaxima(curvepoint_shapely, boundary_shapely, [], "idxminus")
        # ax2.plot([maxima_xminus.x], [maxima_xminus.y], 'yo', markersize = 10)

        # # curvepointbreaking_shapely = polygonlist[2].centroid
        # # ax2.plot([curvepointbreaking_shapely.x], [curvepointbreaking_shapely.y], 'ko', markersize = 30)
        #
        # verts = list(boundary_shapely.coords)
        # index = -1
        # mindist = self.plusinf
        # for i in range(0, len(verts)):
        #     # print Point(verts[i][0], verts[i][1]).distance(curvepoint_shapely)
        #     # ax2.plot([verts[i][0]], [verts[i][1]], 'yo', markersize = 10)
        #     dist = Point(verts[i][0], verts[i][1]).distance(curvepoint_shapely)
        #     if dist < mindist:
        #         mindist = dist
        #         index = i
        # # ax2.plot([verts[index][0]], [verts[index][1]], 'ro', markersize = 35)
        # for i in range(0,100):
        #     idxplus = (index + i) % len(verts)
        #     ax2.plot([verts[idxplus][0]], [verts[idxplus][1]], 'go', markersize = 15)
        #     # print verts[idxplus][1], curvepoint_shapely.y
        #     if verts[idxplus][1] < curvepoint_shapely.y and math.fabs(verts[idxplus][1]-curvepoint_shapely.y) > 10:
        #         break

        plt.show()
        # pass
        fig = plt.gcf()
        # compute orientation of grip
        angle = math.atan2(grppoint1.y-grppoint0.y, grppoint1.x-grppoint0.x)
        angle = math.degrees(angle)
        fig.savefig(str(angle)+'_'+str(grppoint0)+','+str(grppoint1)+'.png')

# Thefollowing code is deprecated
# if __name__=="__main__":
#
#     polygon = Polygon([[0.0,0.0],
#                        [1000.0,0.0],
#                        [2000.0,1000.0],
#                        [1000.0,1500.0],
#                        [0.0,1000.0]])
#
#     sppoint0 = Point(1440.0,820.0)
#     # sppoint0 = Point(-487.0,1160.0)
#     sppoint1 = Point(900.0,0.0)
#     sppoint2 = Point(0.0,100.0)
#
#     # polygon = Polygon([[185.4,0.0],
#     #                    [785.4,0.0],
#     #                    [970.8,570.63],
#     #                    [485.4,923.3],
#     #                    [0,570.63]])
#     #
#     # sppoint0 = Point(970, 600.0)
#     # sppoint1 = Point(92.7, 285.3)
#     # sppoint2 = Point(878.1, 285.3)
#
#
#     d01 = (sppoint0.x-sppoint1.x)*(sppoint0.x-sppoint1.x) + (sppoint0.y-sppoint1.y)*(sppoint0.y-sppoint1.y)
#     d02 = (sppoint0.x-sppoint2.x)*(sppoint0.x-sppoint2.x) + (sppoint0.y-sppoint2.y)*(sppoint0.y-sppoint2.y)
#     if d01>d02:
#         constraindirect01 = [sppoint0.x+sppoint1.x-2*sppoint2.x, sppoint0.y+sppoint1.y-2*sppoint2.y]
#         constraindirect02 = [2*sppoint1.x-sppoint0.x-sppoint2.x, 2*sppoint1.y-sppoint0.y-sppoint2.y]
#     else:
#         constraindirect01 = [2*sppoint2.x-sppoint0.x-sppoint1.x, 2*sppoint2.y-sppoint0.y-sppoint1.y]
#         constraindirect02 = [sppoint0.x+sppoint2.x-2*sppoint1.x, sppoint0.y+sppoint2.y-2*sppoint1.y]
#
#     scale = 10.0
#     steplength = 11
#     [onedcurves01, polygonxoncurves01, fingerp0s3d, fingerp1s3d] = \
#         self.stb.genOneDCurve(polygon, sppoint0, sppoint1, constraindirect = constraindirect01, scale = scale, steplength = steplength)
#     [onedcurves02, polygonxoncurves02, fingerp0s3d, fingerp2s3d] = \
#         self.stb.genOneDCurve(polygon, sppoint0, sppoint2, constraindirect = constraindirect02, scale = scale, steplength = steplength)
#     [onedcurves12, polygonxoncurves12, fingerp1s3d, fingerp2s3d] = \
#         self.stb.genOneDCurve(polygon, sppoint1, sppoint2, scale = scale, steplength = steplength)
#
#     # plot
#     polygoncenter = [polygon.centroid.x, polygon.centroid.y, 360*scale/2.0]
#     base = pc.World(camp=[-10000,4000,2500], lookatp=polygoncenter)
#
#     # plot cobt
#     plotCobt(base.render, sppoint0, polygon, scale, steplength)
#     plotCobt(base.render, sppoint1, polygon, scale, steplength, rgba = [.5,1,.5,1])
#     plotCobt(base.render, sppoint2, polygon, scale, steplength, rgba = [1,.5,.5,1])
#
#     onedcurvedict01 = cvtcurveList2Dict(onedcurves01, scale, steplength)
#     onedcurvedict02 = cvtcurveList2Dict(onedcurves02, scale, steplength)
#     onedcurvedict12 = cvtcurveList2Dict(onedcurves12, scale, steplength)
#
#     polygonxoncurvedict01 = cvtpolygonList2Dict(polygonxoncurves01, onedcurves01, scale, steplength)
#     polygonxoncurvedict02 = cvtpolygonList2Dict(polygonxoncurves02, onedcurves02, scale, steplength)
#     polygonxoncurvedict12 = cvtpolygonList2Dict(polygonxoncurves12, onedcurves12, scale, steplength)
#
#     cobtsdict0 = self.stb.genCobt(polygon, sppoint0, scale=scale, steplength=steplength)
#     cobtsdict1 = self.stb.genCobt(polygon, sppoint1, scale=scale, steplength=steplength)
#     cobtsdict2 = self.stb.genCobt(polygon, sppoint2, scale=scale, steplength=steplength)
#     #
#     onedcurvedict12 = cvtcurveList2Dict(onedcurves12, scale, steplength)
#     onedcurvedict12, polygonxoncurvedict12 = removeCollisiononOnedCurves(onedcurvedict12, cobtsdict0, polygonxoncurvedict12)
#     newonedcurves12 = cvtcurveDict2List(onedcurvedict12, scale, steplength)
#     newpolygonxoncurves12 = cvtpolygonDict2List(polygonxoncurvedict12, scale, steplength)
#     #
#     onedcurvedict01 = cvtcurveList2Dict(onedcurves01, scale, steplength)
#     onedcurvedict01, polygonxoncurvedict01 = removeCollisiononOnedCurves(onedcurvedict01, cobtsdict2, polygonxoncurvedict01)
#     newonedcurves01 = cvtcurveDict2List(onedcurvedict01, scale, steplength)
#     newpolygonxoncurves01 = cvtpolygonDict2List(polygonxoncurvedict01, scale, steplength)
#     #
#     onedcurvedict02 = cvtcurveList2Dict(onedcurves02, scale, steplength)
#     onedcurvedict02, polygonxoncurvedict02 = removeCollisiononOnedCurves(onedcurvedict02, cobtsdict1, polygonxoncurvedict02)
#     newonedcurves02 = cvtcurveDict2List(onedcurvedict02, scale, steplength)
#     newpolygonxoncurves02 = cvtpolygonDict2List(polygonxoncurvedict02, scale, steplength)
#
#     # plot maxima
#     for i, onedcurve in enumerate(newonedcurves12):
#         onedcavities = self.stb.cavities(onedcurve)
#         for cavity in onedcavities:
#             pg.plotSphere(base.render, onedcurve[cavity[0]], radius = 100, rgba = [1,1,0,.5])
#             pg.plotSphere(base.render, onedcurve[cavity[2]], radius = 100, rgba = [1,1,0,.5])
#
#     for i, onedcurve in enumerate(newonedcurves01):
#         onedcavities = self.stb.cavities(onedcurve)
#         for cavity in onedcavities:
#             pg.plotSphere(base.render, onedcurve[cavity[0]], radius = 100, rgba = [1,1,0,.5])
#             pg.plotSphere(base.render, onedcurve[cavity[2]], radius = 100, rgba = [1,1,0,.5])
#
#     for i, onedcurve in enumerate(newonedcurves02):
#         onedcavities = self.stb.cavities(onedcurve)
#         for cavity in onedcavities:
#             pg.plotSphere(base.render, onedcurve[cavity[0]], radius = 100, rgba = [1,1,0,.5])
#             pg.plotSphere(base.render, onedcurve[cavity[2]], radius = 100, rgba = [1,1,0,.5])
#
#     currentpolygon0=[]
#     for i, polygonxoncurve in enumerate(polygonxoncurves12):
#         for j, polygonx in enumerate(polygonxoncurve):
#             if j == 106:
#                 currentpolygon0 = polygonx
#                 # height = onedcurves12[i][j][2]
#                 # polygonsnp = self.stb.genPolygonsnp(polygonx, height, color=[0.0, 0.5, 0.0, 1], thickness=20)
#                 # for polygonnp in polygonsnp:
#                 #     polygonnp.reparentTo(base.render)
#
#     currentpolygon1=[]
#     for i, polygonxoncurve in enumerate(newpolygonxoncurves01):
#         for j, polygonx in enumerate(polygonxoncurve):
#             if i==0 and j == 102:
#                 currentpolygon1 = polygonx
#                 height = newonedcurves01[i][j][2]
#                 polygonsnp = self.stb.genPolygonsnp(polygonx, height, color=[0.5, 0.5, 0.0, 1], thickness=20)
#                 for polygonnp in polygonsnp:
#                     polygonnp.reparentTo(base.render)
#
#     # currentpolygon2=[]
#     # for i, polygonxoncurve in enumerate(newpolygonxoncurves02):
#     #     for j, polygonx in enumerate(polygonxoncurve):
#     #         if i==1 and j == 8:
#     #             currentpolygon2 = polygonx
#                 # height = newonedcurves02[i][j][2]
#                 # polygonsnp = self.stb.genPolygonsnp(polygonx, height, color=[0.5, 0.5, 0.0, 1], thickness=20)
#                 # for polygonnp in polygonsnp:
#                 #     polygonnp.reparentTo(base.render)
#
#     for i, onedcurve in enumerate(onedcurves01):
#         pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 1.0, 1], thickness=10)
#     for i, onedcurve in enumerate(newonedcurves01):
#         pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 1.0, 1], thickness=50)
#     for i, onedcurve in enumerate(onedcurves02):
#         pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 0.0, 1.0, 1], thickness=10)
#     for i, onedcurve in enumerate(newonedcurves02):
#         pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, .0, 1.0, 1], thickness=50)
#     for i, onedcurve in enumerate(onedcurves12):
#         pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 0.5, 0.0, 1], thickness=10)
#         onedcavities = self.stb.cavities(onedcurve)
#         for cavity in onedcavities:
#             pg.plotSphere(base.render, onedcurve[cavity[0]], radius = 100, rgba = [.5,0,0,1])
#             pg.plotSphere(base.render, onedcurve[cavity[1]], radius = 100, rgba = [0,0,.5,1])
#             pg.plotSphere(base.render, onedcurve[cavity[2]], radius = 100, rgba = [.5,0,0,1])
#     for i, onedcurve in enumerate(newonedcurves12):
#         pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, .5, 0.0, 1], thickness=50)
#
#     for i, fingerp03d in enumerate(fingerp0s3d):
#         pg.plotLinesegs(base.render, fingerp03d, rgba=[1.0, 0.0, 0.0, 1], thickness=30)
#     for i, fingerp13d in enumerate(fingerp1s3d):
#         pg.plotLinesegs(base.render, fingerp13d, rgba=[0.0, 0.5, 0.0, 1], thickness=30)
#     for i, fingerp23d in enumerate(fingerp2s3d):
#         pg.plotLinesegs(base.render, fingerp23d, rgba=[0.0, 0.5, 0.0, 1], thickness=30)
#
#
#     def plot2d(polygon, sppoint0, sppoint1, sppoint2, polygonfacecolor = 'g'):
#         fig = plt.figure()
#         ax2 = fig.add_subplot(111)
#         xpolygon, ypolygon = polygon.exterior.xy
#         ax2.fill(xpolygon, ypolygon, alpha=1, fc=polygonfacecolor, ec='none')
#         ax2.plot([sppoint0.x], [sppoint0.y], 'ro')
#         ax2.plot([sppoint1.x], [sppoint1.y], 'ko')
#         ax2.plot([sppoint2.x], [sppoint2.y], 'ko')
#         verts = polygon.exterior.coords
#         ax2.plot([polygon.centroid.x], [polygon.centroid.y], 'bo')
#         ax2.axis('equal')
#         ax2.axis([-500, 1700, -500, 2000])
#
#         plt.show()
#         pass
#
#     # thread.start_new_thread(plot2d, (currentpolygon0, sppoint0, sppoint1, sppoint2,))
#     thread.start_new_thread(plot2d, (currentpolygon1, sppoint0, sppoint1, sppoint2, 'y',))
#     # thread.start_new_thread(plot2d, (currentpolygon2, sppoint0, sppoint1, sppoint2, 'y',))
#
#     pg.plotAxisSelf(base.render, length=1500, thickness = 70)
#     base.run()