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

import mergecurves as mgc

if __name__=="__main__":

    polygon = Polygon([[0.0,0.0],
                       [1000.0,0.0],
                       [2000.0,1000.0],
                       [1000.0,1500.0],
                       [0.0,1000.0]])

    # sppoint0 = Point(1300.0,200.0)
    sppoint0 = Point(1200.0,980.0)
    sppoint1 = Point(900.0,0.0)
    sppoint2 = Point(0.0,100.0)

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

    cobtsdict0 = stb.genCobt(polygon, sppoint0, scale=scale, steplength=steplength)
    cobtsdict1 = stb.genCobt(polygon, sppoint1, scale=scale, steplength=steplength)
    cobtsdict2 = stb.genCobt(polygon, sppoint2, scale=scale, steplength=steplength)
    #
    onedcurvedict12 = mgc.cvtcurveList2Dict(onedcurves12, scale, steplength)
    onedcurvedict12 = mgc.removeCollisiononOnedCurves(onedcurvedict12, cobtsdict0)
    newonedcurves12 = mgc.cvtcurveDict2List(onedcurvedict12, scale, steplength)
    #
    onedcurvedict01 = mgc.cvtcurveList2Dict(onedcurves01, scale, steplength)
    onedcurvedict01 = mgc.removeCollisiononOnedCurves(onedcurvedict01, cobtsdict2)
    newonedcurves01 = mgc.cvtcurveDict2List(onedcurvedict01, scale, steplength)
    #
    onedcurvedict02 = mgc.cvtcurveList2Dict(onedcurves02, scale, steplength)
    onedcurvedict02 = mgc.removeCollisiononOnedCurves(onedcurvedict02, cobtsdict1)
    newonedcurves02 = mgc.cvtcurveDict2List(onedcurvedict02, scale, steplength)

    # plot
    polygoncenter = [polygon.centroid.x, polygon.centroid.y, 360*scale/2.0]
    base = pc.World(camp=[-5000,2000,10000], lookatp=polygoncenter)

    cobtsp0 = stb.genMinkovPnt(polygon, sppoint0)
    for height in range(0,int(360*scale),steplength):
        rotedcobsp0 = stb.rotate(cobtsp0, height/scale, sppoint0)
        rotedcobsp0polygonsnp = stb.genPolygonsnp(rotedcobsp0, height, color = [0.3,.3,.3,.6], thickness=1)
        for polygonnp in rotedcobsp0polygonsnp:
            polygonnp.reparentTo(base.render)

    for i, onedcurve in enumerate(onedcurves01):
        pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 1.0, 1], thickness=30)
    for i, onedcurve in enumerate(newonedcurves01):
        pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, .5, 1.0, 1], thickness=50)
    for i, onedcurve in enumerate(onedcurves02):
        pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 0.0, 1.0, 1], thickness=30)
    for i, onedcurve in enumerate(newonedcurves02):
        pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, .5, 1.0, 1], thickness=50)
    for i, onedcurve in enumerate(onedcurves12):
        onedcavities = stb.cavities(onedcurve)
        for cavity in onedcavities:
            # pg.plotLinesegs(base.render, onedcurve[cavity[0]:cavity[2]+1], rgba=[0.0, 1.0, 0.0, 1], thickness=50)
            pg.plotSphere(base.render, onedcurve[cavity[1]], radius = 100, rgba = [0,0,1,1])
            pg.plotSphere(base.render, onedcurve[cavity[0]], radius = 100, rgba = [1,0,0,1])
            pg.plotSphere(base.render, onedcurve[cavity[2]], radius = 100, rgba = [1,0,0,1])
        pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 0.5, 0.0, 1], thickness=30)
    for i, onedcurve in enumerate(newonedcurves12):
        pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, .5, 1.0, 1], thickness=50)

    for i, fingerp03d in enumerate(fingerp0s3d):
        pg.plotLinesegs(base.render, fingerp03d, rgba=[1.0, 0.0, 0.0, 1], thickness=30)
    for i, fingerp13d in enumerate(fingerp1s3d):
        pg.plotLinesegs(base.render, fingerp13d, rgba=[0.0, 0.5, 0.0, 1], thickness=30)
    for i, fingerp23d in enumerate(fingerp2s3d):
        pg.plotLinesegs(base.render, fingerp23d, rgba=[0.0, 0.5, 0.0, 1], thickness=30)

    objcolorarray = pg.randomColorArray(int(360*scale/float(steplength))+1, alpha = .2)
    # for i, polygonxoncurve in enumerate(polygonxoncurves01):
    #     for j, polygonx in enumerate(polygonxoncurve):
    #         height = onedcurves01[i][j][2]
    #         polygonsnp = stb.genPolygonsnp(polygonx, height, color=[1.0, 1.0, 0.0, .2], thickness=20)
    #         for polygonnp in polygonsnp:
    #             polygonnp.reparentTo(base.render)
    # for i, polygonxoncurve in enumerate(polygonxoncurves02):
    #     for j, polygonx in enumerate(polygonxoncurve):
    #         height = onedcurves02[i][j][2]
    #         polygonsnp = stb.genPolygonsnp(polygonx, height, color=[1.0, 0.0, 1.0, .2], thickness=20)
    #         for polygonnp in polygonsnp:
    #             polygonnp.reparentTo(base.render)
    currentpolygon = None
    for i, polygonxoncurve in enumerate(polygonxoncurves12):
        for j, polygonx in enumerate(polygonxoncurve):
            if j == 106:
                currentpolygon = polygonx
                height = onedcurves12[i][j][2]
                polygonsnp = stb.genPolygonsnp(polygonx, height, color=[0.0, 0.5, 0.0, 1], thickness=20)
                for polygonnp in polygonsnp:
                    polygonnp.reparentTo(base.render)


    def plot2d(polygon, sppoint0, sppoint1, sppoint2):
        import matplotlib.pyplot as plt
        fig = plt.figure()
        ax2 = fig.add_subplot(111)
        xpolygon, ypolygon = polygon.exterior.xy
        ax2.fill(xpolygon, ypolygon, alpha=1, fc='g', ec='none')
        ax2.plot([sppoint0.x], [sppoint0.y], 'ro')
        ax2.plot([sppoint1.x], [sppoint1.y], 'ko')
        ax2.plot([sppoint2.x], [sppoint2.y], 'ko')
        ax2.axis('equal')
        ax2.axis([-500, 1700, -500, 2000])

        plt.show()
        pass

    thread.start_new_thread(plot2d, (currentpolygon, sppoint0, sppoint1, sppoint2,))

    pg.plotAxisSelf(base.render, length=1500, thickness = 70)
    base.run()