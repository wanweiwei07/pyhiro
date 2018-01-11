#!/usr/bin/python

from shapely.geometry import Polygon
from shapely.geometry import Point
import os
import math
from panda3d.core import *
import pandaplotutils.pandageom as pg
import pandaplotutils.pandactrl as pc
import matplotlib.pyplot as plt
import numpy as np
import utils.robotmath as rm
import time

import thread

from mergecurves import Mergecurves as Mgc
from stance.stability import Stability as Stb


if __name__=="__main__":


    # polygon = Polygon([[500.0,0.0],
    #                    [1000.0,0.0],
    #                    [1000.0,1500.0],
    #                    [1500.0,1500.0],
    #                    [1500.0,2000.0],
    #                    [0.0,2000.0],
    #                    [0.0,1500.0],
    #                    [500.0,1500.0],
    #                    [500.0,0.0]])

    polygon = Polygon([[0.0,0.0],
                       [1000.0,0.0],
                       [2000.0,1000.0],
                       [1000.0,1500.0],
                       [0.0,1000.0]])

    # plot
    scale = 10.0
    steplength = 50
    rotrange = 480.0
    mgc = Mgc(rotrange, scale, steplength)

    polygoncenter = [polygon.centroid.x, polygon.centroid.y, rotrange*scale/2.0]
    base = pc.World(camp=[7000,15000,0], lookatp=polygoncenter, up=[-1,0,0])
    up = [0, 1, 0]

    # assumption:
    # we assume the object could only escape from the exit formed by grppoint0-grppoint1
    # we assume grppoint0 and grppoint1 are higher than sppoint0 and sppoint1

    # exp3
    # grppoint0 = Point(-700.0,1220.0)
    # grppoint1 = Point(1440.0,620.0)

    # grppoint0 = Point(-593.0,1190.0)
    # grppoint1 = Point(1333.0,650.0)
    #
    # grppoint0 = Point(-486.0,1160.0)
    # grppoint1 = Point(1226.0,680.0)
    #
    # grppoint0 = Point(-379.0,1130.0)
    # grppoint1 = Point(1119.0,710.0)

    sppoint0 = Point(0.0, 100.0)
    sppoint1 = Point(900.0, 0.0)

    # polygon = Polygon([[185.4,0.0],
    #                    [785.4,0.0],
    #                    [970.8,570.63],
    #                    [485.4,923.3],
    #                    [0,570.63]])
    #
    # sppoint0 = Point(970, 600.0)
    # sppoint1 = Point(92.7, 285.3)
    # sppoint2 = Point(878.1, 285.3)

    v_s0 = np.array([sppoint0.x, sppoint0.y])
    v_s1 = np.array([sppoint1.x, sppoint1.y])

    # generate 1 curve
    [onedcurves_s0s1, polygonxoncurves_s0s1, sppoint0_3d, sppoint1_3d] = \
        mgc.stb.genOneDCurve(polygon, sppoint0, sppoint1, constraindirect = up)

    # plot cobt
    mgc.plotCobt(base.render, sppoint0, polygon, rgba = [.6,.6,.6,.9])
    mgc.plotCobt(base.render, sppoint1, polygon, rgba = [.6,.6,.6,.9])
    # mgc.plotCobtDis(base.render, sppoint0, polygon, rgba = [.6,.6,.6,.9])
    # mgc.plotCobtDis(base.render, sppoint1, polygon, rgba = [.6,.6,.6,.9])


    # convert multiple curves (segmented) to a dictionary
    onedcurvedict_s0s1 = mgc.cvtcurveList2Dict(onedcurves_s0s1)
    # convert multiple polygons on the multiple curves to a dictionary
    polygonxoncurvedict_s0s1 = mgc.cvtpolygonList2Dict(polygonxoncurves_s0s1, onedcurves_s0s1)

    cobtsdict_s0 = mgc.stb.genCobt(polygon, sppoint0)
    cobtsdict_s1 = mgc.stb.genCobt(polygon, sppoint1)

    ## remove collided curves (dummy operation)
    onedcurves_s0s1_cdfree = mgc.cvtcurveDict2List(onedcurvedict_s0s1)
    polygonxoncurves_s0s1_cdfree = mgc.cvtpolygonDict2List(polygonxoncurvedict_s0s1)
    #
    # # plot maxima
    # # s0s1
    for i, onedcurve in enumerate(onedcurves_s0s1_cdfree):
        onedcavities = mgc.stb.cavities(onedcurve)
        for j, cavity in enumerate(onedcavities):
            if j == 2:
                # pg.plotSphere(base.render, onedcurve[cavity[0]], radius = 200, rgba = [0,0,1,1])
                pg.plotSphere(base.render, onedcurve[cavity[1]], radius = 200, rgba = [1,0,0,1])
                # pg.plotSphere(base.render, onedcurve[cavity[2]], radius = 200, rgba = [0,0,1,1])
    # s0s1
    for i, onedcurve in enumerate(onedcurves_s0s1_cdfree):
        pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 0.0, 0.0, 1], thickness=100)
    # for i, onedcurve in enumerate(onedcurves_s0s1_cdfree):
    #     onedcavities = mgc.stb.cavities(onedcurve)
    #     for j, cavity in enumerate(onedcavities):
    #         # if j == 2:
    #         pg.plotLinesegs(base.render, onedcurve[cavity[0]:cavity[2]], rgba=[0.0, 0.0, 0.0, 1], thickness=100)
    # startingconf = []
    # breakingconf = []
    lowestconfatbreaking = []
    currentkey = 0
    currentpolygon0=[]
    for i, polygonxoncurve in enumerate(polygonxoncurves_s0s1):
        for j, polygonx in enumerate(polygonxoncurve):
            # start from the middle point
            if math.fabs(onedcurves_s0s1[i][j][2]-mgc.heightrange/2.0) < steplength:
                currentkey = onedcurves_s0s1[i][j][2]
                # print currentkey
                currentpolygon0 = polygonx
                currentconf = onedcurves_s0s1[i][j]
                # pg.plotSphere(base.render, onedcurves_s0s1[i][j], radius = 300, rgba=[1,0,0,1])
                # height = onedcurves_s0s1[i][j][2]
                # polygonsnp = mgc.stb.genPolygonsnp(polygonx, height, color=[0.0, 0.5, 0.0, 1], thickness=20)
                # for polygonnp in polygonsnp:
                #     polygonnp.reparentTo(base.render)
    #  print currentkey
    curvepoint = onedcurvedict_s0s1[currentkey]
    pg.plotSphere(base.render, curvepoint, radius=200, rgba=[1, 1, 0, 1])

    # breaking maxima
    rgtmaximacand = []
    lftmaximacand = []
    for i, onedcurve in enumerate(onedcurves_s0s1_cdfree):
        onedcavities = mgc.stb.cavities(onedcurve)
        for j, cavity in enumerate(onedcavities):
            # print cavity
            if j < 2:
                lftmaximacand.append(onedcurve[cavity[0]])
                lftmaximacand.append(onedcurve[cavity[2]])
            if j > 2:
                rgtmaximacand.append(onedcurve[cavity[0]])
                rgtmaximacand.append(onedcurve[cavity[2]])
            if j == 2:
                lftmaximacand.append(onedcurve[cavity[0]])
                rgtmaximacand.append(onedcurve[cavity[2]])
        # print lftmaximacand
        # print rgtmaximacand
        if len(lftmaximacand) == 0 or len(rgtmaximacand)==0:
            continue
        lftmaxima = max(lftmaximacand, key=lambda item: item[1])
        rgtmaxima = max(rgtmaximacand, key=lambda item: item[1])
        pg.plotSphere(base.render, lftmaxima, radius=200, rgba=[0, 1, 0, 1])
        pg.plotSphere(base.render, rgtmaxima, radius=200, rgba=[0, 1, 0, 1])

    lowestonedcurve = {}
    for key in range(0, mgc.heightrange, mgc.steplength):
        lowestonedcurve[key] = onedcurvedict_s0s1[key]
    #
    #
    breakingmaximas = {}
    # lowestconfs = {}
    # breakingmaximas[0] = breakingconfiguration
    # lowestconfs[0] = bottomconfiguration
    # # # minus 180
    for i in range(-1, -int(180*scale/float(steplength)), -1):
        nxtangle = i*steplength
        key = currentkey+nxtangle
        if key > lftmaxima[2]:
            curvepoint = lowestonedcurve[key]
            polylist_onecobtslice = [cobtsdict_s0[key], cobtsdict_s1[key]]
            cobtsliceunion = polylist_onecobtslice[0]
            for j in range(1, len(polylist_onecobtslice)):
                cobtsliceunion = cobtsliceunion.union(polylist_onecobtslice[j])
            breakingmaxima, breakingconfiguration, valleyminima, bottomconfiguration = \
                mgc.lstb.maxmin_oneslice(curvepoint, polylist_onecobtslice, [])
            breakingmaximas[i] = breakingconfiguration
            # pg.plotSphere(base.render, breakingconfiguration, radius = 200, rgba=[.3,0,.3,1])
    for i in range(1, int(180*scale/float(steplength)), 1):
        nxtangle = i*steplength
        key = currentkey+nxtangle
        if key < rgtmaxima[2]:
            curvepoint = lowestonedcurve[key]
            polylist_onecobtslice = [cobtsdict_s0[key], cobtsdict_s1[key]]
            cobtsliceunion = polylist_onecobtslice[0]
            for j in range(1, len(polylist_onecobtslice)):
                cobtsliceunion = cobtsliceunion.union(polylist_onecobtslice[j])
            breakingmaxima, breakingconfiguration, valleyminima, bottomconfiguration = \
                mgc.lstb.maxmin_oneslice(curvepoint, polylist_onecobtslice, [])
            breakingmaximas[i] = breakingconfiguration
            # pg.plotSphere(base.render, breakingconfiguration, radius = 200, rgba=[.3,0,.3,1])
    print breakingmaximas
    print [item for item in breakingmaximas.values()]
    minbreak = min([item for item in breakingmaximas.values()], key = lambda item:item[1])
    print minbreak
    pg.plotSphere(base.render, minbreak, radius = 200, rgba=[0,1,1,1])
    #
    # startingconf = breakingmaximas[0]
    # breakingconf = breakingmaximas[0]
    # # find breaking
    # #minus
    # breakingconf_minus = breakingmaximas[0]
    # lastmaxbreaking_minus = lowestconfs[0]
    # globallowestbreaking_minus = [mgc.plusinf, mgc.plusinf]
    # isfullvalley = True
    # for i in range(-1, -int(180*scale/float(steplength)), -1):
    #     if breakingmaximas[i][1] <= lowestconfs[i][1]+1.0:
    #         isfullvalley = False
    #         break
    #     if lowestconfs[i][1] > lastmaxbreaking_minus[1]:
    #         lastmaxbreaking_minus = lowestconfs[i]
    #     if breakingmaximas[i][1] < globallowestbreaking_minus[1]:
    #         globallowestbreaking_minus = breakingmaximas[i]
    # if isfullvalley:
    #     breakingconf_minus = globallowestbreaking_minus
    # else:
    #     breakingconf_minus = lastmaxbreaking_minus
    # #plus
    # breakingconf_plus = breakingmaximas[0]
    # lastmaxbreaking_plus = lowestconfs[0]
    # globallowestbreaking_plus = [mgc.plusinf, mgc.plusinf]
    # isfullvalley = True
    # for i in range(1, int(180*scale/float(steplength)), 1):
    #     if breakingmaximas[i][1] <= lowestconfs[i][1]:
    #         isfullvalley = False
    #         break
    #     if lowestconfs[i][1] > lastmaxbreaking_plus[1]:
    #         lastmaxbreaking_plus = lowestconfs[i]
    #     if breakingmaximas[i][1] < globallowestbreaking_plus[1]:
    #         globallowestbreaking_plus = breakingmaximas[i]
    # if isfullvalley:
    #     breakingconf_plus = globallowestbreaking_plus
    # else:
    #     breakingconf_plus = lastmaxbreaking_plus
    #
    # breakingconf = min([breakingconf_minus, breakingconf_plus], key = lambda item:item[1])
    # # global breaking conf
    # # pg.plotSphere(base.render, breakingconf_minus, radius=300, rgba=[1, 1, 0, 1])
    # # pg.plotSphere(base.render, breakingconf_plus, radius=300, rgba=[1, 1, 0, 1])
    # # pg.plotSphere(base.render, breakingconf, radius=300, rgba=[1, 1, 0, 1])
    #
    # # nearest lowest conf
    # startingconf_minus = breakingconf
    # for i in range(-1, -int(180*scale/float(steplength)), -1):
    #     if lowestconfs[i][1] >= startingconf_minus[1]:
    #         break
    #     else:
    #         startingconf_minus = lowestconfs[i]
    # startingconf_plus = breakingconf
    # for i in range(1, int(180*scale/float(steplength)), 1):
    #     if lowestconfs[i][1] >= startingconf_plus[1]:
    #         break
    #     else:
    #         startingconf_plus = lowestconfs[i]
    # startingconf = min([startingconf_minus, startingconf_plus], key = lambda item:item[1])
    # pg.plotSphere(base.render, startingconf, radius = 300, rgba=[1,1,0,1])
    #
    # #
    # # currentpolygon1=[]
    # # for i, polygonxoncurve in enumerate(newpolygonxoncurves01):
    # #     for j, polygonx in enumerate(polygonxoncurve):
    # #         if i==0 and j == 102:
    # #             currentpolygon1 = polygonx
    # #             height = newonedcurves01[i][j][2]
    # #             polygonsnp = mgc.stb.genPolygonsnp(polygonx, height, color=[0.5, 0.5, 0.0, 1], thickness=20)
    # #             for polygonnp in polygonsnp:
    # #                 polygonnp.reparentTo(base.render)
    #
    # # currentpolygon2=[]
    # # for i, polygonxoncurve in enumerate(newpolygonxoncurves02):
    # #     for j, polygonx in enumerate(polygonxoncurve):
    # #         if i==1 and j == 8:
    # #             currentpolygon2 = polygonx
    #             # height = newonedcurves02[i][j][2]
    #             # polygonsnp = mgc.stb.genPolygonsnp(polygonx, height, color=[0.5, 0.5, 0.0, 1], thickness=20)
    #             # for polygonnp in polygonsnp:
    #             #     polygonnp.reparentTo(base.render)
    #
    # # g0s0
    # # for i, onedcurve in enumerate(onedcurves_g0s0):
    # #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 1.0, 1], thickness=10)
    # # for i, onedcurve in enumerate(onedcurves_g0s0_cdfree):
    # #     pg.plotLinesegs(base.render, onedcurve, rgba=[.5, 0.0, 0.5, 1], thickness=150)
    # # # pg.plotArrow(base.render, spos = np.array([0,0,0]), epos=np.array([0,0,0])+np.array([constraindirect_g0s0[0], constraindirect_g0s0[1], 0]), length = 10000)
    # # # g0s1
    # # for i, onedcurve in enumerate(onedcurves_g0s1):
    # #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 0.0, 1.0, 1], thickness=10)
    # # for i, onedcurve in enumerate(onedcurves_g0s1_cdfree):
    # #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 0.0, 1.0, 1], thickness=150)
    # # # g0g1
    # # for i, onedcurve in enumerate(onedcurves_g0g1):
    # #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 1.0, 0.0, 1], thickness=10)
    # # for i, onedcurve in enumerate(onedcurves_g0g1_cdfree):
    # #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 1.0, 0.0, 1], thickness=150)
    # # # s0s1
    # # for i, onedcurve in enumerate(onedcurves_s0s1):
    # #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 1.0, 1.0, 1], thickness=10)
    # # for i, onedcurve in enumerate(onedcurves_s0s1_cdfree):
    # #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, .6, 0.0, 1], thickness=150)
    # # # s0g1
    # # for i, onedcurve in enumerate(onedcurves_s0g1):
    # #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 0.0, 1], thickness=10)
    # # for i, onedcurve in enumerate(onedcurves_s0g1_cdfree):
    # #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 0.0, 1], thickness=150)
    # # # s1g1
    # # for i, onedcurve in enumerate(onedcurves_s1g1):
    # #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 0.0, 0.0, 1], thickness=10)
    # # for i, onedcurve in enumerate(onedcurves_s1g1_cdfree):
    # #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.5, 0.0, 0.5, 1], thickness=150)
    #
    # # for i, grp_3d in enumerate(grppoint0_3d):
    # #     pg.plotLinesegs(base.render, grp_3d, rgba=[1.0, 0.0, 0.0, 1], thickness=30)
    # # for i, grp_3d in enumerate(grppoint1_3d):
    # #     pg.plotLinesegs(base.render, grp_3d, rgba=[1.0, 0.0, 0.0, 1], thickness=30)
    # # for i, sp_3d in enumerate(sppoint0_3d):
    # #     pg.plotLinesegs(base.render, sp_3d, rgba=[0.0, 0.0, 1.0, 1], thickness=30)
    # # for i, sp_3d in enumerate(sppoint1_3d):
    # #     pg.plotLinesegs(base.render, sp_3d, rgba=[0.0, 0.0, 1.0, 1], thickness=30)
    # #
    # # print breakingconf[1] - startingconf[1]
    # # breakingpolygon = mgc.stb.genPolygonFromConf(polygon, breakingconf)
    # # #
    # # # # thread.start_new_thread(mgc.plot2d, (polygon, grppoint0, grppoint1, sppoint0, sppoint1))
    # # # # lowestpolygonatbreaking = mgc.stb.genPolygonFromConf(polygon, lowestconfatbreaking)
    # # lowestpolygonatbreaking = breakingpolygon
    # # # # thread.start_new_thread(mgc.plot2d, (breakingpolygon, grppoint0, grppoint1, sppoint0, sppoint1, 'y'))
    # # thread.start_new_thread(mgc.plot2d_list, ([currentpolygon0, lowestpolygonatbreaking, breakingpolygon,
    # #                                            cobtsdict_g0[breakingconf[2]], cobtsdict_g1[breakingconf[2]],
    # #                                            cobtsdict_s0[breakingconf[2]], cobtsdict_s1[breakingconf[2]]],
    # #                                           grppoint0, grppoint1, sppoint0, sppoint1, ['r', 'y', 'y', 'g', 'g', 'k', 'k']))
    # startingpolygon = mgc.stb.genPolygonFromConf(polygon, startingconf)
    # #
    # # # thread.start_new_thread(mgc.plot2d, (polygon, grppoint0, grppoint1, sppoint0, sppoint1))
    # # # lowestpolygonatbreaking = mgc.stb.genPolygonFromConf(polygon, lowestconfatbreaking)
    # lowestpolygonatstart = startingpolygon
    # # # thread.start_new_thread(mgc.plot2d, (breakingpolygon, grppoint0, grppoint1, sppoint0, sppoint1, 'y'))
    # thread.start_new_thread(mgc.plot2d_list, ([currentpolygon0, lowestpolygonatstart, startingpolygon,
    #                                            cobtsdict_g0[startingconf[2]], cobtsdict_g1[startingconf[2]],
    #                                            cobtsdict_s0[startingconf[2]], cobtsdict_s1[startingconf[2]]],
    #                                           grppoint0, grppoint1, sppoint0, sppoint1, ['r', 'y', 'y', 'g', 'g', 'k', 'k']))
    # # for i, polygonxoncurve in enumerate(polygonxoncurves_s0s1):
    # #     for j, polygonx in enumerate(polygonxoncurve):
    # #         # start from the middle point
    # #         if math.fabs(onedcurves_s0s1[i][j][2]-mgc.heightrange/2.0) < steplength:
    # #             currentconf = onedcurves_s0s1[i][j]
    # # currentpolygon = mgc.stb.genPolygonFromConf(polygon, currentconf)
    # # #
    # # # # thread.start_new_thread(mgc.plot2d, (polygon, grppoint0, grppoint1, sppoint0, sppoint1))
    # # # # lowestpolygonatbreaking = mgc.stb.genPolygonFromConf(polygon, lowestconfatbreaking)
    # # polygonatstart = currentpolygon
    # # # # thread.start_new_thread(mgc.plot2d, (breakingpolygon, grppoint0, grppoint1, sppoint0, sppoint1, 'y'))
    # # thread.start_new_thread(mgc.plot2d_list, ([currentpolygon0, polygonatstart, currentpolygon,
    # #                                            cobtsdict_g0[currentconf[2]], cobtsdict_g1[currentconf[2]],
    # #                                            cobtsdict_s0[currentconf[2]], cobtsdict_s1[currentconf[2]]],
    # #                                           grppoint0, grppoint1, sppoint0, sppoint1, ['r', 'y', 'y', 'g', 'g', 'k', 'k']))
    # #
    # # pg.plotAxisSelf(base.render, length=1500, thickness = 70)
    base.run()