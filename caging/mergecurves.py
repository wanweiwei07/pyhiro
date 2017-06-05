#!/usr/bin/python

from shapely.geometry import Polygon
from shapely.geometry import Point
import stance.stability as stb
import os
import math
from panda3d.core import *
import pandaplotutils.pandageom as pg
import pandaplotutils.pandactrl as pc
import thread
import matplotlib.pyplot as plt

def cvtcurveList2Dict(onedcurves, scale, steplength):
    onedcurvedict = {}
    minusinf = -9999999999
    heightrange = int(360 * scale)
    for i in range(0, heightrange, steplength):
        onedcurvedict[i] = [minusinf, minusinf, i]
    for i, onedcurve in enumerate(onedcurves):
        for j, point in enumerate(onedcurve):
            onedcurvedict[point[2]] = point
    return onedcurvedict

def cvtcurveDict2List(onedcurvedict, scale, steplength):
    onedcurves = []
    onedcurve = []
    heightrange = int(360 * scale)
    for i in range(0, heightrange, steplength):
        if onedcurvedict[i][0] < -9999999998:
            if len(onedcurve) != 0:
                onedcurves.append(onedcurve)
                onedcurve = []
        elif i > heightrange - steplength:
            onedcurves.append(onedcurve)
            onedcurve = []
        else:
            onedcurve.append(onedcurvedict[i])
    return onedcurves

def cvtpolygonList2Dict(polygonxoncurves, onedcurves, scale, steplength):
    polygonxoncurvedict = {}
    heightrange = int(360 * scale)
    for i in range(0, heightrange, steplength):
        polygonxoncurvedict[i] = None
    for i, polygonxoncurve in enumerate(polygonxoncurves):
        for j, polygonx in enumerate(polygonxoncurve):
            height = onedcurves[i][j][2]
            polygonxoncurvedict[height] = polygonx
    return polygonxoncurvedict

def cvtpolygonDict2List(polygonxoncurvedict, scale, steplength):
    polygonxoncurves = []
    polygonxoncurve = []
    heightrange = int(360 * scale)
    for i in range(0, heightrange, steplength):
        if polygonxoncurvedict[i] is None:
            if len(polygonxoncurve) != 0:
                polygonxoncurves.append(polygonxoncurve)
                polygonxoncurve = []
        elif i > heightrange - steplength:
            polygonxoncurves.append(polygonxoncurve)
            polygonxoncurve = []
        else:
            polygonxoncurve.append(polygonxoncurvedict[i])
    return polygonxoncurves

def removeCollisiononOnedCurves(onedcurvedict, collidingPolygondict, polygonxoncurvedict):
    """
    remove the nodes that collide with collidinPolygondict
    :param onedcurvedict: the curve for collision test
    :param collidingPolygondict: the polygon for collision detection
    :param polygonxoncurvedict: the curvedict to be updated
    :return: an updated oned curve dict

    author: weiwei
    date: 20170512
    """
    newonedcurvedict = {}
    newpolygonxoncurvedict = {}
    minusinf = -9999999999
    for key, point in onedcurvedict.iteritems():
        newonedcurvedict[key] = [minusinf, minusinf, key]
        newpolygonxoncurvedict[key] = None
        polygonx = collidingPolygondict[key]
        if not polygonx.contains(Point(point[0], point[1])):
            newonedcurvedict[key] = point
            newpolygonxoncurvedict[key] = polygonxoncurvedict[key]

    return [newonedcurvedict, newpolygonxoncurvedict]

def plotCobt(nodepath, point, polygon, scale, steplength, rgba = [.5, .5, .5, 1]):
    """
    generate the configuration obstacle

    :param point:
    :param polygon:
    :return:

    author: weiwei
    date: 20170517
    """

    # plot
    cobt = stb.genMinkovPnt(polygon, point)
    polygonlist = []
    for height in range(0,int(360*scale),steplength):
        rotedcobt = stb.rotate(cobt, height/scale)
        polygonlist.append(rotedcobt)
    cobtnp = stb.genCobtnp(polygonlist, steplength)
    cobtnp.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
    cobtnp.setTransparency(TransparencyAttrib.MAlpha)
    cobtnp.reparentTo(nodepath)

def plotCobtSeg(nodepath, segment, polygon, scale, steplength, rgba = [.5, .5, .5, 1]):
    """
    TODO WRONG! segments do not confirm!
    generate the configuration obstacle

    :param point:
    :param polygon:
    :return:

    author: weiwei
    date: 20170517
    """

    # plot
    polygonlist = []
    for height in range(0,int(360*scale),steplength):
        rotedcobt = stb.genMinkovSeg(stb.rotate(polygon, height/scale), segment)
        polygonlist.append(rotedcobt)
    cobtnp = stb.genCobtnp(polygonlist, steplength)
    cobtnp.setColor(rgba[0], rgba[1], rgba[2], rgba[3])
    cobtnp.setTransparency(TransparencyAttrib.MAlpha)
    cobtnp.reparentTo(nodepath)

if __name__=="__main__":

    polygon = Polygon([[0.0,0.0],
                       [1000.0,0.0],
                       [2000.0,1000.0],
                       [1000.0,1500.0],
                       [0.0,1000.0]])

    sppoint0 = Point(1440.0,820.0)
    # sppoint0 = Point(-487.0,1160.0)
    sppoint1 = Point(900.0,0.0)
    sppoint2 = Point(0.0,100.0)

    # polygon = Polygon([[185.4,0.0],
    #                    [785.4,0.0],
    #                    [970.8,570.63],
    #                    [485.4,923.3],
    #                    [0,570.63]])
    #
    # sppoint0 = Point(970, 600.0)
    # sppoint1 = Point(92.7, 285.3)
    # sppoint2 = Point(878.1, 285.3)


    d01 = (sppoint0.x-sppoint1.x)*(sppoint0.x-sppoint1.x) + (sppoint0.y-sppoint1.y)*(sppoint0.y-sppoint1.y)
    d02 = (sppoint0.x-sppoint2.x)*(sppoint0.x-sppoint2.x) + (sppoint0.y-sppoint2.y)*(sppoint0.y-sppoint2.y)
    if d01>d02:
        constraindirect01 = [sppoint0.x+sppoint1.x-2*sppoint2.x, sppoint0.y+sppoint1.y-2*sppoint2.y]
        constraindirect02 = [2*sppoint1.x-sppoint0.x-sppoint2.x, 2*sppoint1.y-sppoint0.y-sppoint2.y]
    else:
        constraindirect01 = [2*sppoint2.x-sppoint0.x-sppoint1.x, 2*sppoint2.y-sppoint0.y-sppoint1.y]
        constraindirect02 = [sppoint0.x+sppoint2.x-2*sppoint1.x, sppoint0.y+sppoint2.y-2*sppoint1.y]

    scale = 10.0
    steplength = 11
    [onedcurves01, polygonxoncurves01, fingerp0s3d, fingerp1s3d] = \
        stb.genOneDCurve(polygon, sppoint0, sppoint1, constraindirect = constraindirect01, scale = scale, steplength = steplength)
    [onedcurves02, polygonxoncurves02, fingerp0s3d, fingerp2s3d] = \
        stb.genOneDCurve(polygon, sppoint0, sppoint2, constraindirect = constraindirect02, scale = scale, steplength = steplength)
    [onedcurves12, polygonxoncurves12, fingerp1s3d, fingerp2s3d] = \
        stb.genOneDCurve(polygon, sppoint1, sppoint2, scale = scale, steplength = steplength)

    # plot
    polygoncenter = [polygon.centroid.x, polygon.centroid.y, 360*scale/2.0]
    base = pc.World(camp=[-10000,4000,2500], lookatp=polygoncenter)

    # plot cobt
    plotCobt(base.render, sppoint0, polygon, scale, steplength)
    plotCobt(base.render, sppoint1, polygon, scale, steplength, rgba = [.5,1,.5,1])
    plotCobt(base.render, sppoint2, polygon, scale, steplength, rgba = [1,.5,.5,1])

    onedcurvedict01 = cvtcurveList2Dict(onedcurves01, scale, steplength)
    onedcurvedict02 = cvtcurveList2Dict(onedcurves02, scale, steplength)
    onedcurvedict12 = cvtcurveList2Dict(onedcurves12, scale, steplength)

    polygonxoncurvedict01 = cvtpolygonList2Dict(polygonxoncurves01, onedcurves01, scale, steplength)
    polygonxoncurvedict02 = cvtpolygonList2Dict(polygonxoncurves02, onedcurves02, scale, steplength)
    polygonxoncurvedict12 = cvtpolygonList2Dict(polygonxoncurves12, onedcurves12, scale, steplength)

    cobtsdict0 = stb.genCobt(polygon, sppoint0, scale=scale, steplength=steplength)
    cobtsdict1 = stb.genCobt(polygon, sppoint1, scale=scale, steplength=steplength)
    cobtsdict2 = stb.genCobt(polygon, sppoint2, scale=scale, steplength=steplength)
    #
    onedcurvedict12 = cvtcurveList2Dict(onedcurves12, scale, steplength)
    onedcurvedict12, polygonxoncurvedict12 = removeCollisiononOnedCurves(onedcurvedict12, cobtsdict0, polygonxoncurvedict12)
    newonedcurves12 = cvtcurveDict2List(onedcurvedict12, scale, steplength)
    newpolygonxoncurves12 = cvtpolygonDict2List(polygonxoncurvedict12, scale, steplength)
    #
    onedcurvedict01 = cvtcurveList2Dict(onedcurves01, scale, steplength)
    onedcurvedict01, polygonxoncurvedict01 = removeCollisiononOnedCurves(onedcurvedict01, cobtsdict2, polygonxoncurvedict01)
    newonedcurves01 = cvtcurveDict2List(onedcurvedict01, scale, steplength)
    newpolygonxoncurves01 = cvtpolygonDict2List(polygonxoncurvedict01, scale, steplength)
    #
    onedcurvedict02 = cvtcurveList2Dict(onedcurves02, scale, steplength)
    onedcurvedict02, polygonxoncurvedict02 = removeCollisiononOnedCurves(onedcurvedict02, cobtsdict1, polygonxoncurvedict02)
    newonedcurves02 = cvtcurveDict2List(onedcurvedict02, scale, steplength)
    newpolygonxoncurves02 = cvtpolygonDict2List(polygonxoncurvedict02, scale, steplength)

    # plot maxima
    for i, onedcurve in enumerate(newonedcurves12):
        onedcavities = stb.cavities(onedcurve)
        for cavity in onedcavities:
            pg.plotSphere(base.render, onedcurve[cavity[0]], radius = 100, rgba = [1,1,0,.5])
            pg.plotSphere(base.render, onedcurve[cavity[2]], radius = 100, rgba = [1,1,0,.5])

    for i, onedcurve in enumerate(newonedcurves01):
        onedcavities = stb.cavities(onedcurve)
        for cavity in onedcavities:
            pg.plotSphere(base.render, onedcurve[cavity[0]], radius = 100, rgba = [1,1,0,.5])
            pg.plotSphere(base.render, onedcurve[cavity[2]], radius = 100, rgba = [1,1,0,.5])

    for i, onedcurve in enumerate(newonedcurves02):
        onedcavities = stb.cavities(onedcurve)
        for cavity in onedcavities:
            pg.plotSphere(base.render, onedcurve[cavity[0]], radius = 100, rgba = [1,1,0,.5])
            pg.plotSphere(base.render, onedcurve[cavity[2]], radius = 100, rgba = [1,1,0,.5])

    currentpolygon0=[]
    for i, polygonxoncurve in enumerate(polygonxoncurves12):
        for j, polygonx in enumerate(polygonxoncurve):
            if j == 106:
                currentpolygon0 = polygonx
                # height = onedcurves12[i][j][2]
                # polygonsnp = stb.genPolygonsnp(polygonx, height, color=[0.0, 0.5, 0.0, 1], thickness=20)
                # for polygonnp in polygonsnp:
                #     polygonnp.reparentTo(base.render)

    currentpolygon1=[]
    for i, polygonxoncurve in enumerate(newpolygonxoncurves01):
        for j, polygonx in enumerate(polygonxoncurve):
            if i==0 and j == 102:
                currentpolygon1 = polygonx
                height = newonedcurves01[i][j][2]
                polygonsnp = stb.genPolygonsnp(polygonx, height, color=[0.5, 0.5, 0.0, 1], thickness=20)
                for polygonnp in polygonsnp:
                    polygonnp.reparentTo(base.render)

    # currentpolygon2=[]
    # for i, polygonxoncurve in enumerate(newpolygonxoncurves02):
    #     for j, polygonx in enumerate(polygonxoncurve):
    #         if i==1 and j == 8:
    #             currentpolygon2 = polygonx
                # height = newonedcurves02[i][j][2]
                # polygonsnp = stb.genPolygonsnp(polygonx, height, color=[0.5, 0.5, 0.0, 1], thickness=20)
                # for polygonnp in polygonsnp:
                #     polygonnp.reparentTo(base.render)

    for i, onedcurve in enumerate(onedcurves01):
        pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 1.0, 1], thickness=10)
    for i, onedcurve in enumerate(newonedcurves01):
        pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 1.0, 1], thickness=50)
    for i, onedcurve in enumerate(onedcurves02):
        pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 0.0, 1.0, 1], thickness=10)
    for i, onedcurve in enumerate(newonedcurves02):
        pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, .0, 1.0, 1], thickness=50)
    for i, onedcurve in enumerate(onedcurves12):
        pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 0.5, 0.0, 1], thickness=10)
        onedcavities = stb.cavities(onedcurve)
        for cavity in onedcavities:
            pg.plotSphere(base.render, onedcurve[cavity[0]], radius = 100, rgba = [.5,0,0,1])
            pg.plotSphere(base.render, onedcurve[cavity[1]], radius = 100, rgba = [0,0,.5,1])
            pg.plotSphere(base.render, onedcurve[cavity[2]], radius = 100, rgba = [.5,0,0,1])
    for i, onedcurve in enumerate(newonedcurves12):
        pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, .5, 0.0, 1], thickness=50)

    for i, fingerp03d in enumerate(fingerp0s3d):
        pg.plotLinesegs(base.render, fingerp03d, rgba=[1.0, 0.0, 0.0, 1], thickness=30)
    for i, fingerp13d in enumerate(fingerp1s3d):
        pg.plotLinesegs(base.render, fingerp13d, rgba=[0.0, 0.5, 0.0, 1], thickness=30)
    for i, fingerp23d in enumerate(fingerp2s3d):
        pg.plotLinesegs(base.render, fingerp23d, rgba=[0.0, 0.5, 0.0, 1], thickness=30)


    def plot2d(polygon, sppoint0, sppoint1, sppoint2, polygonfacecolor = 'g'):
        fig = plt.figure()
        ax2 = fig.add_subplot(111)
        xpolygon, ypolygon = polygon.exterior.xy
        ax2.fill(xpolygon, ypolygon, alpha=1, fc=polygonfacecolor, ec='none')
        ax2.plot([sppoint0.x], [sppoint0.y], 'ro')
        ax2.plot([sppoint1.x], [sppoint1.y], 'ko')
        ax2.plot([sppoint2.x], [sppoint2.y], 'ko')
        verts = polygon.exterior.coords
        ax2.plot([polygon.centroid.x], [polygon.centroid.y], 'bo')
        ax2.axis('equal')
        ax2.axis([-500, 1700, -500, 2000])

        plt.show()
        pass

    # thread.start_new_thread(plot2d, (currentpolygon0, sppoint0, sppoint1, sppoint2,))
    thread.start_new_thread(plot2d, (currentpolygon1, sppoint0, sppoint1, sppoint2, 'y',))
    # thread.start_new_thread(plot2d, (currentpolygon2, sppoint0, sppoint1, sppoint2, 'y',))

    pg.plotAxisSelf(base.render, length=1500, thickness = 70)
    base.run()