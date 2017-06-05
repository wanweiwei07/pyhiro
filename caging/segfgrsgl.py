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
    segment0 = [Point(1413,385), Point(1413,885)]
    sppoint1 = Point(900.0,0.0)
    sppoint2 = Point(0.0,100.0)

    scale = 10.0
    steplength = 55
    # segment 0
    [onedcurvesSeg01, polygonxoncurvesSeg01, segfgr0s3dsp1, sppoint1s3d] = \
        stb.genOneDCurveSeg(polygon, segment0, sppoint1, scale = scale, steplength = steplength)
    [onedcurvesSeg02, polygonxoncurvesSeg02, segfgr0s3dsp2, sppoint2s3d] = \
        stb.genOneDCurveSeg(polygon, segment0, sppoint2, scale = scale, steplength = steplength)
    [onedcurves12, polygonxoncurves12, fingerp1s3d, fingerp2s3d] = \
        stb.genOneDCurve(polygon, sppoint1, sppoint2, scale = scale, steplength = steplength)

    cobtsdictSeg0 = stb.genCobtSeg(polygon, segment0, scale=scale, steplength=steplength)
    cobtsdict1 = stb.genCobt(polygon, sppoint1, scale=scale, steplength=steplength)
    cobtsdict2 = stb.genCobt(polygon, sppoint2, scale=scale, steplength=steplength)

    polygonxoncurvedictSeg01 = mgc.cvtpolygonList2Dict(polygonxoncurvesSeg01, onedcurvesSeg01, scale, steplength)
    polygonxoncurvedictSeg02 = mgc.cvtpolygonList2Dict(polygonxoncurvesSeg02, onedcurvesSeg02, scale, steplength)
    polygonxoncurvedict12 = mgc.cvtpolygonList2Dict(polygonxoncurves12, onedcurves12, scale, steplength)
    # seg0
    onedcurvedictSeg01 = mgc.cvtcurveList2Dict(onedcurvesSeg01, scale, steplength)
    onedcurvedictSeg01, polygonxoncurvedictSeg01 = \
        mgc.removeCollisiononOnedCurves(onedcurvedictSeg01, cobtsdict2, polygonxoncurvedictSeg01)
    newonedcurvesSeg01 = mgc.cvtcurveDict2List(onedcurvedictSeg01, scale, steplength)
    #
    onedcurvedictSeg02 = mgc.cvtcurveList2Dict(onedcurvesSeg02, scale, steplength)
    onedcurvedictSeg02, polygonxoncurvedictSeg02 = \
        mgc.removeCollisiononOnedCurves(onedcurvedictSeg02, cobtsdict1, polygonxoncurvedictSeg02)
    newonedcurvesSeg02 = mgc.cvtcurveDict2List(onedcurvedictSeg02, scale, steplength)
    # fgr12
    onedcurvedict12 = mgc.cvtcurveList2Dict(onedcurves12, scale, steplength)
    onedcurvedict12, polygonxoncurvedict12 = \
        mgc.removeCollisiononOnedCurves(onedcurvedict12, cobtsdictSeg0, polygonxoncurvedict12)
    newonedcurves12 = mgc.cvtcurveDict2List(onedcurvedict12, scale, steplength)

    # plot
    polygoncenter = [polygon.centroid.x, polygon.centroid.y, 360*scale/2.0]
    base = pc.World(camp=[-5000,2000,10000], lookatp=polygoncenter)

    # for height in range(0,int(360*scale),steplength):
    #     rotedcobsegpolygonsnp = stb.genPolygonsnp(cobtsdictSeg0[height], height, color = [0.3,.3,.3,.6], thickness=1)
    #     for polygonnp in rotedcobsegpolygonsnp:
    #         polygonnp.reparentTo(base.render)

    # for height in range(0,int(360*scale),steplength):
    #     rotedcobsegpolygonsnp = stb.genPolygonsnp(cobtsdict1[height], height, color = [0.3,.3,.3,.6], thickness=1)
    #     for polygonnp in rotedcobsegpolygonsnp:
    #         polygonnp.reparentTo(base.render)

    mgc.plotCobt(base.render, sppoint1, polygon, scale, steplength, rgba = [.5,1,.5,1])
    mgc.plotCobt(base.render, sppoint2, polygon, scale, steplength, rgba = [1,.5,.5,1])

    # seg 0 1 seg 0 2
    for i, onedcurve in enumerate(onedcurvesSeg01):
        pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 1.0, 1], thickness=10)
    for i, onedcurve in enumerate(newonedcurvesSeg01):
        pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 1.0, 1], thickness=50)
    for i, onedcurve in enumerate(onedcurvesSeg02):
        pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 0.0, 1.0, 1], thickness=10)
    for i, onedcurve in enumerate(newonedcurvesSeg02):
        pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 0.0, 1.0, 1], thickness=50)
    # 1 2
    for i, onedcurve in enumerate(onedcurves12):
        onedcavities = stb.cavities(onedcurve)
        print onedcavities
        for cavity in onedcavities:
            # pg.plotLinesegs(base.render, onedcurve[cavity[0]:cavity[2]+1], rgba=[0.0, 1.0, 0.0, 1], thickness=50)
            pg.plotSphere(base.render, onedcurve[cavity[1]], radius = 100, rgba = [0,0,1,1])
            pg.plotSphere(base.render, onedcurve[cavity[0]], radius = 100, rgba = [1,0,0,1])
            pg.plotSphere(base.render, onedcurve[cavity[2]], radius = 100, rgba = [1,0,0,1])
        pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 0.5, 0.0, 1], thickness=10)
    for i, onedcurve in enumerate(newonedcurves12):
        pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, .5, 0.0, 1], thickness=50)

    # plot maxima
    # seg 01 02
    for i, onedcurve in enumerate(newonedcurvesSeg01):
        onedcavities = stb.cavities(onedcurve)
        for cavity in onedcavities:
            pg.plotSphere(base.render, onedcurve[cavity[0]], radius = 100, rgba = [1,1,0,.5])
            pg.plotSphere(base.render, onedcurve[cavity[2]], radius = 100, rgba = [1,1,0,.5])
    for i, onedcurve in enumerate(newonedcurvesSeg02):
        onedcavities = stb.cavities(onedcurve)
        for cavity in onedcavities:
            pg.plotSphere(base.render, onedcurve[cavity[0]], radius = 100, rgba = [1,1,0,.5])
            pg.plotSphere(base.render, onedcurve[cavity[2]], radius = 100, rgba = [1,1,0,.5])
    # sp12
    for i, onedcurve in enumerate(newonedcurves12):
        onedcavities = stb.cavities(onedcurve)
        print onedcavities
        for cavity in onedcavities:
            pg.plotSphere(base.render, onedcurve[cavity[0]], radius = 100, rgba = [1,1,0,.5])
            pg.plotSphere(base.render, onedcurve[cavity[2]], radius = 100, rgba = [1,1,0,.5])

    # for i, segfgr3d in enumerate(segfgr0s3dsp1):
    #     for onefgrinsegfgr3d in segfgr3d:
    #         pg.plotLinesegs(base.render, onefgrinsegfgr3d, rgba=[1.0, 0.0, 0.0, 1], thickness=30)
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
            if j == 37:
                currentpolygon = polygonx
                height = onedcurves12[i][j][2]
                polygonsnp = stb.genPolygonsnp(polygonx, height, color=[0.0, 0.5, 0.0, 1], thickness=20)
                for polygonnp in polygonsnp:
                    polygonnp.reparentTo(base.render)


    def plot2d(polygon, segfgr, sppoint1, sppoint2):
        import matplotlib.pyplot as plt
        fig = plt.figure()
        ax2 = fig.add_subplot(111)
        xpolygon, ypolygon = polygon.exterior.xy
        ax2.fill(xpolygon, ypolygon, alpha=1, fc='g', ec='none')
        ax2.plot([segfgr[0].x, segfgr[1].x], [segfgr[0].y, segfgr[1].y], 'r-')
        ax2.plot([sppoint1.x], [sppoint1.y], 'ko')
        ax2.plot([sppoint2.x], [sppoint2.y], 'ko')
        ax2.axis('equal')
        ax2.axis([-500, 1700, -500, 2000])

        plt.show()
        pass

    thread.start_new_thread(plot2d, (currentpolygon, segment0, sppoint1, sppoint2,))

    pg.plotAxisSelf(base.render, length=1500, thickness = 70)
    base.run()