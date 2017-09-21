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

def computeinitialpose(polygon, sppoint0, sppoint1, scale = 10.0, steplength = 30, rotrange = 480.0):
    """

    :param polygon:
    :param sppoint0:
    :param sppoint1:
    :param scale:
    :param steplength:
    :param rotrangle:
    :return:

    author: weiwei
    date: 20170825
    """

    mgc = Mgc(rotrange, scale, steplength)

    v_s0 = np.array([sppoint0.x, sppoint0.y])
    v_s1 = np.array([sppoint1.x, sppoint1.y])

    ## constraindirect_s0s1
    constraindirect_s0s1 = np.array([0.0, 1.0])
    if v_s1[1] != v_s0[1]:
        v_s0s1 = v_s1 - v_s0
        v_s0s1_normalized = v_s0s1 / np.linalg.norm(v_s0s1)
        constraindirect_s0s1 = v_s0 + np.dot(-v_s0, v_s0s1_normalized) * v_s0s1_normalized
        constraindirect_s0s1 = constraindirect_s0s1 / np.linalg.norm(constraindirect_s0s1)

    [onedcurves_s0s1, polygonxoncurves_s0s1, sppoint0_3d, sppoint1_3d] = \
        mgc.stb.genOneDCurve(polygon, sppoint0, sppoint1, constraindirect=constraindirect_s0s1)
    # onedcurvedict_s0s1 = mgc.cvtcurveList2Dict(onedcurves_s0s1)
    # print onedcurvedict_s0s1
    # polygonxoncurvedict_s0s1 = mgc.cvtpolygonList2Dict(polygonxoncurves_s0s1, onedcurves_s0s1)

    for i, polygonxoncurve in enumerate(polygonxoncurves_s0s1):
        for j, polygonx in enumerate(polygonxoncurve):
            if math.fabs(onedcurves_s0s1[i][j][2] - mgc.heightrange / 2.0) < steplength:
                currentkey = onedcurves_s0s1[i][j][2]
                currentpolygon0 = polygonx
                # currentpolygon0 = polygonxoncurvedict_s0s1[currentkey]
                return [currentkey, currentpolygon0]

def computestabilitywithsk(polygon, grppoint0, grppoint1, sppoint0, sppoint1, startingkey, scale = 10.0, steplength = 30, rotrange = 480.0):
    """
    compute the depth of valley
    and the save the breaking states as images

    # assumption:
    # the function assumes the object could only escape from the exit formed by grppoint0-grppoint1
    # grppoint0 and grppoint1 are higher than sppoint0 and sppoint1

    :param polygon:
    :param grppoint0:
    :param grppoint1:
    :param sppoint0:
    :param sppoint1:
    :param startingkey
    :return:
    """

    mgc = Mgc(rotrange, scale, steplength)

    # polygoncenter = [polygon.centroid.x, polygon.centroid.y, rotrange* scale / 2.0]
    # base = pc.World(camp=[-10000, 4000, 2500], lookatp=polygoncenter)

    v_g0 = np.array([grppoint0.x, grppoint0.y])
    v_g1 = np.array([grppoint1.x, grppoint1.y])
    v_s0 = np.array([sppoint0.x, sppoint0.y])
    v_s1 = np.array([sppoint1.x, sppoint1.y])

    v_g0g1 = v_g1 - v_g0
    v_g1g0 = v_g0 - v_g1

    ## constraindirect_g0s0
    if v_s0[1] >= v_g0[1]:
        assert ("grppoint0 should be higher than sppoint0")
    constraindirect_g0s0 = np.array([1.0, 0.0])
    if v_g0[0] != v_s0[0]:
        v_g0s0 = v_s0 - v_g0
        v_g0s0_normalized = v_g0s0 / np.linalg.norm(v_g0s0)
        constraindirect_g0s0 = v_g0 + np.dot(-v_g0, v_g0s0_normalized) * v_g0s0_normalized
        constraindirect_g0s0 = constraindirect_g0s0 / np.linalg.norm(constraindirect_g0s0)
        # pg.plotArrow(base.render, np.array([0,0,0]), (np.array([0,0,0])+np.array([constraindirect_g0s0[0], constraindirect_g0s0[1], 0]))*1000)
        if rm.degree_between(constraindirect_g0s0, v_g0g1) > 90:
            constraindirect_g0s0 = -constraindirect_g0s0
    ## constraindirect_g0s1
    if v_s1[1] >= v_g0[1]:
        assert ("grppoint0 should be higher than sppoint1")
    constraindirect_g0s1 = np.array([1.0, 0.0])
    if v_g0[0] != v_s1[0]:
        v_g0s1 = v_s1 - v_g0
        v_g0s1_normalized = v_g0s1 / np.linalg.norm(v_g0s1)
        constraindirect_g0s1 = v_g0 + np.dot(-v_g0, v_g0s1_normalized) * v_g0s1_normalized
        constraindirect_g0s1 = constraindirect_g0s1 / np.linalg.norm(constraindirect_g0s1)
        if rm.degree_between(constraindirect_g0s1, v_g0g1) > 90:
            constraindirect_g0s1 = -constraindirect_g0s1
    ## constraindirect_g0g1
    constraindirect_g0g1 = np.array([0.0, 1.0])
    if v_g0[1] != v_g1[1]:
        v_g0g1 = v_g1 - v_g0
        v_g0g1_normalized = v_g0g1 / np.linalg.norm(v_g0g1)
        constraindirect_g0g1 = v_g0 + np.dot(-v_g0, v_g0g1_normalized) * v_g0g1_normalized
        constraindirect_g0g1 = constraindirect_g0g1 / np.linalg.norm(constraindirect_g0g1)
    ## constraindirect_s0s1
    constraindirect_s0s1 = np.array([0.0, 1.0])
    if v_s1[1] != v_s0[1]:
        v_s0s1 = v_s1 - v_s0
        v_s0s1_normalized = v_s0s1 / np.linalg.norm(v_s0s1)
        constraindirect_s0s1 = v_s0 + np.dot(-v_s0, v_s0s1_normalized) * v_s0s1_normalized
        constraindirect_s0s1 = constraindirect_s0s1 / np.linalg.norm(constraindirect_s0s1)
    ## constraindirect_s0g1
    if v_s0[1] >= v_g1[1]:
        assert ("grppoint1 should be higher than sppoint0")
    constraindirect_g1s0 = np.array([-1.0, 0.0])
    if v_g1[0] != v_s0[0]:
        v_s0g1 = v_g1 - v_s0
        v_s0g1_normalized = v_s0g1 / np.linalg.norm(v_s0g1)
        constraindirect_s0g1 = v_s0 + np.dot(-v_s0, v_s0g1_normalized) * v_s0g1_normalized
        constraindirect_s0g1 = constraindirect_s0g1 / np.linalg.norm(constraindirect_s0g1)
        if rm.degree_between(constraindirect_s0g1, v_g1g0) > 90:
            constraindirect_s0g1 = -constraindirect_s0g1
    ## constraindirect_s1g1
    if v_s1[1] >= v_g1[1]:
        assert ("grppoint1 should be higher than sppoint1")
    constraindirect_g1s1 = np.array([-1.0, 0.0])
    if v_g1[0] != v_s1[0]:
        v_s1g1 = v_g1 - v_s1
        v_s1g1_normalized = v_s1g1 / np.linalg.norm(v_s1g1)
        constraindirect_s1g1 = v_s1 + np.dot(-v_s1, v_s1g1_normalized) * v_s1g1_normalized
        constraindirect_s1g1 = constraindirect_s1g1 / np.linalg.norm(constraindirect_s1g1)
        if rm.degree_between(constraindirect_s1g1, v_g1g0) > 90:
            constraindirect_s1g1 = -constraindirect_s1g1

    # add exit
    # constraindirect_g0s0 = constraindirect_g0s0+constraindirect_g0g1
    # constraindirect_g0s1 = constraindirect_g0s1+constraindirect_g0g1
    # constraindirect_s0g1 = constraindirect_s0g1+constraindirect_g0g1
    # constraindirect_s1g1 = constraindirect_s1g1+constraindirect_g0g1

    # generate 6 curves
    # curve 1: grppoint0-sppoint0 curves
    [onedcurves_g0s0, polygonxoncurves_g0s0, grppoint0_3d, sppoint0_3d] = \
        mgc.stb.genOneDCurve(polygon, grppoint0, sppoint0, constraindirect=constraindirect_g0s0)
    # curve 2: grppoint0-sppoint1 curve
    [onedcurves_g0s1, polygonxoncurves_g0s1, grppoint0_3d, sppoint1_3d] = \
        mgc.stb.genOneDCurve(polygon, grppoint0, sppoint1, constraindirect=constraindirect_g0s1)
    # curve 3: grppoint0-grppoint1 curve
    [onedcurves_g0g1, polygonxoncurves_g0g1, grppoint0_3d, grppoint1_3d] = \
        mgc.stb.genOneDCurve(polygon, grppoint0, grppoint1, constraindirect=constraindirect_g0g1)
    # curve 4: sppoint0-sppoint1 curve
    [onedcurves_s0s1, polygonxoncurves_s0s1, sppoint0_3d, sppoint1_3d] = \
        mgc.stb.genOneDCurve(polygon, sppoint0, sppoint1, constraindirect=constraindirect_s0s1)
    # curve 5: sppoint0-grppoint1 curve
    [onedcurves_s0g1, polygonxoncurves_s0g1, sppoint0_3d, grppoint1_3d] = \
        mgc.stb.genOneDCurve(polygon, sppoint0, grppoint1, constraindirect=constraindirect_s0g1)
    # curve 6: sppoint1-grppoint1 curve
    [onedcurves_s1g1, polygonxoncurves_s1g1, sppoint1_3d, grppoint1_3d] = \
        mgc.stb.genOneDCurve(polygon, sppoint1, grppoint1, constraindirect=constraindirect_s1g1)

    # plot cobt
    # color of finger 0 => red
    # mgc.plotCobt(base.render, grppoint0, polygon, rgba=[.7, .3, .3, .7])
    # color of finger 1 => blue
    # mgc.plotCobt(base.render, grppoint1, polygon, rgba=[.3, .3, .7, .7])
    # color of supports => gray
    # mgc.plotCobt(base.render, sppoint0, polygon, rgba=[.5, .5, .5, .5])
    # mgc.plotCobt(base.render, sppoint1, polygon, rgba=[.5, .5, .5, .5])

    # convert multiple curves (segmented) to a dictionary
    onedcurvedict_g0s0 = mgc.cvtcurveList2Dict(onedcurves_g0s0)
    onedcurvedict_g0s1 = mgc.cvtcurveList2Dict(onedcurves_g0s1)
    onedcurvedict_g0g1 = mgc.cvtcurveList2Dict(onedcurves_g0g1)
    onedcurvedict_s0s1 = mgc.cvtcurveList2Dict(onedcurves_s0s1)
    onedcurvedict_s0g1 = mgc.cvtcurveList2Dict(onedcurves_s0g1)
    onedcurvedict_s1g1 = mgc.cvtcurveList2Dict(onedcurves_s1g1)
    # convert multiple polygons on the multiple curves to a dictionary
    polygonxoncurvedict_g0s0 = mgc.cvtpolygonList2Dict(polygonxoncurves_g0s0, onedcurves_g0s0)
    polygonxoncurvedict_g0s1 = mgc.cvtpolygonList2Dict(polygonxoncurves_g0s1, onedcurves_g0s1)
    polygonxoncurvedict_g0g1 = mgc.cvtpolygonList2Dict(polygonxoncurves_g0g1, onedcurves_g0g1)
    polygonxoncurvedict_s0s1 = mgc.cvtpolygonList2Dict(polygonxoncurves_s0s1, onedcurves_s0s1)
    polygonxoncurvedict_s0g1 = mgc.cvtpolygonList2Dict(polygonxoncurves_s0g1, onedcurves_s0g1)
    polygonxoncurvedict_s1g1 = mgc.cvtpolygonList2Dict(polygonxoncurves_s1g1, onedcurves_s1g1)

    cobtsdict_g0 = mgc.stb.genCobt(polygon, grppoint0)
    cobtsdict_g1 = mgc.stb.genCobt(polygon, grppoint1)
    cobtsdict_s0 = mgc.stb.genCobt(polygon, sppoint0)
    cobtsdict_s1 = mgc.stb.genCobt(polygon, sppoint1)

    ## remove collided curves between curve_g0s0 and obt g1, obt s1
    onedcurvedict_g0s0_rg1, polygonxoncurvedict_g0s0_rg1 = \
        mgc.removeCollisiononOnedCurves(onedcurvedict_g0s0, polygonxoncurvedict_g0s0, cobtsdict_g1)
    onedcurvedict_g0s0_rg1s1, polygonxoncurvedict_g0s0_rg1s1 = \
        mgc.removeCollisiononOnedCurves(onedcurvedict_g0s0_rg1, polygonxoncurvedict_g0s0_rg1, cobtsdict_s1)
    onedcurves_g0s0_cdfree = mgc.cvtcurveDict2List(onedcurvedict_g0s0_rg1s1)
    polygonxoncurves_g0s0_cdfree = mgc.cvtpolygonDict2List(polygonxoncurvedict_g0s0_rg1s1)
    ## remove collided curves between curve_g0s1 and obt g1, obt s0
    onedcurvedict_g0s1_rg1, polygonxoncurvedict_g0s1_rg1 = \
        mgc.removeCollisiononOnedCurves(onedcurvedict_g0s1, polygonxoncurvedict_g0s1, cobtsdict_g1)
    onedcurvedict_g0s1_rg1s0, polygonxoncurvedict_g0s1_rg1s0 = \
        mgc.removeCollisiononOnedCurves(onedcurvedict_g0s1_rg1, polygonxoncurvedict_g0s1_rg1, cobtsdict_s0)
    onedcurves_g0s1_cdfree = mgc.cvtcurveDict2List(onedcurvedict_g0s1_rg1s0)
    polygonxoncurves_g0s1_cdfree = mgc.cvtpolygonDict2List(polygonxoncurvedict_g0s1_rg1s0)
    ## remove collided curves between curve_g0g1 and obt s0, obt s1
    onedcurvedict_g0g1_rs0, polygonxoncurvedict_g0g1_rs0 = \
        mgc.removeCollisiononOnedCurves(onedcurvedict_g0g1, polygonxoncurvedict_g0g1, cobtsdict_s0)
    onedcurvedict_g0g1_rs0s1, polygonxoncurvedict_g0g1_rs0s1 = \
        mgc.removeCollisiononOnedCurves(onedcurvedict_g0g1_rs0, polygonxoncurvedict_g0g1_rs0, cobtsdict_s1)
    onedcurves_g0g1_cdfree = mgc.cvtcurveDict2List(onedcurvedict_g0g1_rs0s1)
    polygonxoncurves_g0g1_cdfree = mgc.cvtpolygonDict2List(polygonxoncurvedict_g0g1_rs0s1)
    # remove collided curves between curve_s0s1 and obt g0, obt g1
    onedcurvedict_s0s1_rg0, polygonxoncurvedict_s0s1_rg0 = \
        mgc.removeCollisiononOnedCurves(onedcurvedict_s0s1, polygonxoncurvedict_s0s1, cobtsdict_g0)
    onedcurvedict_s0s1_rg0g1, polygonxoncurvedict_s0s1_rg0g1 = \
        mgc.removeCollisiononOnedCurves(onedcurvedict_s0s1_rg0, polygonxoncurvedict_s0s1_rg0, cobtsdict_g1)
    onedcurves_s0s1_cdfree = mgc.cvtcurveDict2List(onedcurvedict_s0s1_rg0g1)
    polygonxoncurves_s0s1_cdfree = mgc.cvtpolygonDict2List(polygonxoncurvedict_s0s1_rg0g1)
    ## remove collided curves between curve_s0g1 and obt s1, obt g0
    onedcurvedict_s0g1_rs1, polygonxoncurvedict_s0g1_rs1 = \
        mgc.removeCollisiononOnedCurves(onedcurvedict_s0g1, polygonxoncurvedict_s0g1, cobtsdict_s1)
    onedcurvedict_s0g1_rs1g0, polygonxoncurvedict_s0g1_rs1g0 = \
        mgc.removeCollisiononOnedCurves(onedcurvedict_s0g1_rs1, polygonxoncurvedict_s0g1_rs1, cobtsdict_g0)
    onedcurves_s0g1_cdfree = mgc.cvtcurveDict2List(onedcurvedict_s0g1_rs1g0)
    polygonxoncurves_s0g1_cdfree = mgc.cvtpolygonDict2List(polygonxoncurvedict_s0g1_rs1g0)
    ## remove collided curves between curve_s1g1 and obt g0, obt s0
    onedcurvedict_s1g1_rg0, polygonxoncurvedict_s1g1_rg0 = \
        mgc.removeCollisiononOnedCurves(onedcurvedict_s1g1, polygonxoncurvedict_s1g1, cobtsdict_g0)
    onedcurvedict_s1g1_rg0s0, polygonxoncurvedict_s1g1_rg0s0 = \
        mgc.removeCollisiononOnedCurves(onedcurvedict_s1g1_rg0, polygonxoncurvedict_s1g1_rg0, cobtsdict_s0)
    onedcurves_s1g1_cdfree = mgc.cvtcurveDict2List(onedcurvedict_s1g1_rg0s0)
    polygonxoncurves_s1g1_cdfree = mgc.cvtpolygonDict2List(polygonxoncurvedict_s1g1_rg0s0)

    # plot maxima
    # g0s0
    # for i, onedcurve in en
    # ity[2]], radius=100, rgba=[1, 1, 0, .5])

    # # startingconf = []
    # # breakingconf = []
    # # lowestconfatbreaking = []
    # # the initial configuration is assumed to be half of heighrange
    # currentkey = 0
    # currentpolygon0 = []
    # for i, polygonxoncurve in enumerate(polygonxoncurves_s0s1):
    #     for j, polygonx in enumerate(polygonxoncurve):
    #         if math.fabs(onedcurves_s0s1[i][j][2] - mgc.heightrange / 2.0) < steplength:
    #             currentkey = onedcurves_s0s1[i][j][2]
    #             currentpolygon0 = polygonx
    #             currentconf = onedcurves_s0s1[i][j]
    #             # pg.plotSphere(base.render, onedcurves_s0s1[i][j], radius=300, rgba=[1, 0, 0, 1])
    #             # height = onedcurves_s0s1[i][j][2]
    #             # polygonsnp = mgc.stb.genPolygonsnp(polygonx, height, color=[0.0, 0.5, 0.0, 1], thickness=20)
    #             # for polygonnp in polygonsnp:
    #             #     polygonnp.reparentTo(base.render)
    currentkey = startingkey
    # print currentkey
    curvepoint = onedcurvedict_s0s1[currentkey]
    currentpolygon0 = polygonxoncurvedict_s0s1[currentkey]
    # print currentpolygon0
    # print onedcurvedict_s0s1
    polylist_onecobtslice = [cobtsdict_g0[currentkey], cobtsdict_s0[currentkey],
                             cobtsdict_s1[currentkey], cobtsdict_g1[currentkey]]
    othercurvepoints = []
    a = onedcurvedict_g0s0_rg1s1[currentkey]
    b = onedcurvedict_g0s1_rg1s0[currentkey]
    c = onedcurvedict_g0g1_rs0s1[currentkey]
    d = onedcurvedict_s0s1_rg0g1[currentkey]
    e = onedcurvedict_s0g1_rs1g0[currentkey]
    f = onedcurvedict_s1g1_rg0s0[currentkey]
    if a[1] > mgc.minusinf + 1:
        othercurvepoints.append([a[0], a[1], currentkey])
    if b[1] > mgc.minusinf + 1:
        othercurvepoints.append([b[0], b[1], currentkey])
    if c[1] > mgc.minusinf + 1:
        othercurvepoints.append([c[0], c[1], currentkey])
    if d[1] > mgc.minusinf + 1:
        othercurvepoints.append([d[0], d[1], currentkey])
    if e[1] > mgc.minusinf + 1:
        othercurvepoints.append([e[0], e[1], currentkey])
    if f[1] > mgc.minusinf + 1:
        othercurvepoints.append([f[0], f[1], currentkey])
    breakingmaxima, breakingconfiguration, valleyminima, bottomconfiguration = \
        mgc.lstb.maxmin_oneslice(curvepoint, polylist_onecobtslice, othercurvepoints)
    # pg.plotSphere(base.render, breakingconfiguration, radius=200, rgba=[.3, 0, .3, 1])
    # pg.plotSphere(base.render, bottomconfiguration, radius=300, rgba=[1, 0, 0, 1])
    # startingconf = bottomconfiguration
    # breakingconf = breakingconfiguration
    # lowestconfatbreaking = bottomconfiguration

    lowestonedcurve = {}
    # for key in range(0, mgc.heightrange, mgc.steplength):
    #     vert = [mgc.minusinf, mgc.minusinf, key]
    #     a = onedcurvedict_g0s0_rg1s1[key]
    #     b = onedcurvedict_g0s1_rg1s0[key]
    #     c = onedcurvedict_g0g1_rs0s1[key]
    #     d = onedcurvedict_s0s1_rg0g1[key]
    #     e = onedcurvedict_s0g1_rs1g0[key]
    #     f = onedcurvedict_s1g1_rg0s0[key]
    #     # print a,b,c,d,e,f
    #     # time.sleep(1.0)
    #     vertlist = []
    #     if a[1] > mgc.minusinf + 1:
    #         vertlist.append([a[0], a[1], key])
    #     if b[1] > mgc.minusinf + 1:
    #         vertlist.append([b[0], b[1], key])
    #     if c[1] > mgc.minusinf + 1:
    #         vertlist.append([c[0], c[1], key])
    #     if d[1] > mgc.minusinf + 1:
    #         vertlist.append([d[0], d[1], key])
    #     if e[1] > mgc.minusinf + 1:
    #         vertlist.append([e[0], e[1], key])
    #     if f[1] > mgc.minusinf + 1:
    #         vertlist.append([f[0], f[1], key])
    #     if len(vertlist) > 0:
    #         vert = min(vertlist, key=lambda item: item[1])
    #         # print vert
    #     lowestonedcurve[key] = vert
    for key in range(0, mgc.heightrange, mgc.steplength):
        main = onedcurvedict_s0s1_rg0g1[key]
        a = onedcurvedict_g0s0_rg1s1[key]
        b = onedcurvedict_g0s1_rg1s0[key]
        c = onedcurvedict_g0g1_rs0s1[key]
        d = onedcurvedict_s0g1_rs1g0[key]
        e = onedcurvedict_s1g1_rg0s0[key]
        vert = [mgc.minusinf, mgc.minusinf, key]
        if main[1] > mgc.minusinf+1:
            vert = main
        else:
            vertlist = []
            if a[1] > mgc.minusinf + 1:
                vertlist.append([a[0], a[1], key])
            if b[1] > mgc.minusinf + 1:
                vertlist.append([b[0], b[1], key])
            if c[1] > mgc.minusinf + 1:
                vertlist.append([c[0], c[1], key])
            if d[1] > mgc.minusinf + 1:
                vertlist.append([d[0], d[1], key])
            if e[1] > mgc.minusinf + 1:
                vertlist.append([e[0], e[1], key])
            if len(vertlist) > 0:
                vert = min(vertlist, key=lambda item: item[1])
        lowestonedcurve[key] = vert

    breakingmaximas = {}
    lowestconfs = {}
    breakingmaximas[0] = breakingconfiguration
    lowestconfs[0] = bottomconfiguration
    # # minus 180
    for i in range(-1, -int(180 * scale / float(steplength)), -1):
        nxtangle = i * steplength
        key = currentkey + nxtangle
        curvepoint = lowestonedcurve[key]
        if curvepoint[1] > mgc.minusinf+1:
            polylist_onecobtslice = [cobtsdict_g0[key], cobtsdict_s0[key],
                                     cobtsdict_s1[key], cobtsdict_g1[key]]
            othercurvepoints = []
            a = onedcurvedict_g0s0_rg1s1[key]
            b = onedcurvedict_g0s1_rg1s0[key]
            c = onedcurvedict_g0g1_rs0s1[key]
            d = onedcurvedict_s0s1_rg0g1[key]
            e = onedcurvedict_s0g1_rs1g0[key]
            f = onedcurvedict_s1g1_rg0s0[key]
            if a[1] > mgc.minusinf + 1:
                othercurvepoints.append([a[0], a[1], key])
            if b[1] > mgc.minusinf + 1:
                othercurvepoints.append([b[0], b[1], key])
            if c[1] > mgc.minusinf + 1:
                othercurvepoints.append([c[0], c[1], key])
            if d[1] > mgc.minusinf + 1:
                othercurvepoints.append([d[0], d[1], key])
            if e[1] > mgc.minusinf + 1:
                othercurvepoints.append([e[0], e[1], key])
            if f[1] > mgc.minusinf + 1:
                othercurvepoints.append([f[0], f[1], key])
            breakingmaxima, breakingconfiguration, valleyminima, bottomconfiguration = \
                mgc.lstb.maxmin_oneslice(curvepoint, polylist_onecobtslice, othercurvepoints)
            breakingmaximas[i] = breakingconfiguration
            lowestconfs[i] = bottomconfiguration
            # pg.plotSphere(base.render, breakingconfiguration, radius=200, rgba=[.3, 0, .3, 1])
            # pg.plotSphere(base.render, bottomconfiguration, radius=200, rgba=[.3, 0, 0, 1])
            # if breakingconfiguration[1] < breakingconf[1]:
            #     breakingconf = breakingconfiguration
            #     lowestconfatbreaking = bottomconfiguration
    # plus 180
    for i in range(1, int(180 * scale / float(steplength))):
        nxtangle = i * steplength
        key = currentkey + nxtangle
        curvepoint = lowestonedcurve[key]
        if curvepoint[1] > mgc.minusinf+1:
            polylist_onecobtslice = [cobtsdict_g0[key], cobtsdict_s0[key],
                                     cobtsdict_s1[key], cobtsdict_g1[key]]
            othercurvepoints = []
            a = onedcurvedict_g0s0_rg1s1[key]
            b = onedcurvedict_g0s1_rg1s0[key]
            c = onedcurvedict_g0g1_rs0s1[key]
            d = onedcurvedict_s0s1_rg0g1[key]
            e = onedcurvedict_s0g1_rs1g0[key]
            f = onedcurvedict_s1g1_rg0s0[key]
            if a[1] > mgc.minusinf + 1:
                othercurvepoints.append([a[0], a[1], key])
            if b[1] > mgc.minusinf + 1:
                othercurvepoints.append([b[0], b[1], key])
            if c[1] > mgc.minusinf + 1:
                othercurvepoints.append([c[0], c[1], key])
            if d[1] > mgc.minusinf + 1:
                othercurvepoints.append([d[0], d[1], key])
            if e[1] > mgc.minusinf + 1:
                othercurvepoints.append([e[0], e[1], key])
            if f[1] > mgc.minusinf + 1:
                othercurvepoints.append([f[0], f[1], key])
            breakingmaxima, breakingconfiguration, valleyminima, bottomconfiguration = \
                mgc.lstb.maxmin_oneslice(curvepoint, polylist_onecobtslice, othercurvepoints)
            breakingmaximas[i] = breakingconfiguration
            lowestconfs[i] = bottomconfiguration
            # pg.plotSphere(base.render, breakingconfiguration, radius=200, rgba=[.3, 0, .3, 1])
            # pg.plotSphere(base.render, bottomconfiguration, radius=200, rgba=[.3, 0, 0, 1])
            # if breakingconfiguration[1] < breakingconf[1]:
            #     breakingconf = breakingconfiguration
            #     lowestconfatbreaking = bottomconfiguration

    startingconf = breakingmaximas[0]
    breakingconf = breakingmaximas[0]
    # find breaking
    #minus
    breakingconf_minus = breakingmaximas[0]
    lastmaxbreaking_minus = lowestconfs[0]
    globallowestbreaking_minus = [mgc.plusinf, mgc.plusinf]
    isfullvalley = True
    for i in range(-1, -int(180*scale/float(steplength)), -1):
        if breakingmaximas[i][1] <= lowestconfs[i][1]+1.0:
            isfullvalley = False
            break
        if lowestconfs[i][1] > lastmaxbreaking_minus[1]:
            lastmaxbreaking_minus = lowestconfs[i]
        if breakingmaximas[i][1] < globallowestbreaking_minus[1]:
            globallowestbreaking_minus = breakingmaximas[i]
    if isfullvalley:
        breakingconf_minus = globallowestbreaking_minus
    else:
        breakingconf_minus = lastmaxbreaking_minus
    #plus
    breakingconf_plus = breakingmaximas[0]
    lastmaxbreaking_plus = lowestconfs[0]
    globallowestbreaking_plus = [mgc.plusinf, mgc.plusinf]
    isfullvalley = True
    for i in range(1, int(180*scale/float(steplength)), 1):
        if breakingmaximas[i][1] <= lowestconfs[i][1]:
            isfullvalley = False
            break
        if lowestconfs[i][1] > lastmaxbreaking_plus[1]:
            lastmaxbreaking_plus = lowestconfs[i]
        if breakingmaximas[i][1] < globallowestbreaking_plus[1]:
            globallowestbreaking_plus = breakingmaximas[i]
    if isfullvalley:
        breakingconf_plus = globallowestbreaking_plus
    else:
        breakingconf_plus = lastmaxbreaking_plus
    breakingconf = min([breakingconf_minus, breakingconf_plus], key = lambda item:item[1])
    # global breaking conf
    # pg.plotSphere(base.render, breakingconf, radius=500, rgba=[1, 1, 0, 1])

    # nearest lowest conf
    startingconf_minus = breakingconf
    for i in range(-1, -int(180*scale/float(steplength)), -1):
        if lowestconfs[i][1] >= startingconf_minus[1]:
            break
        else:
            startingconf_minus = lowestconfs[i]
    startingconf_plus = breakingconf
    for i in range(1, int(180*scale/float(steplength)), 1):
        if lowestconfs[i][1] >= startingconf_plus[1]:
            break
        else:
            startingconf_plus = lowestconfs[i]
    startingconf = min([startingconf_minus, startingconf_plus], key = lambda item:item[1])

    # # g0s0
    # for i, onedcurve in enumerate(onedcurves_g0s0):
    #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 1.0, 1], thickness=10)
    # for i, onedcurve in enumerate(onedcurves_g0s0_cdfree):
    #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 1.0, 1], thickness=50)
    # # pg.plotArrow(base.render, spos = np.array([0,0,0]), epos=np.array([0,0,0])+np.array([constraindirect_g0s0[0], constraindirect_g0s0[1], 0]), length = 10000)
    # # g0s1
    # for i, onedcurve in enumerate(onedcurves_g0s1):
    #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 0.0, 1.0, 1], thickness=10)
    # for i, onedcurve in enumerate(onedcurves_g0s1_cdfree):
    #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 0.0, 1.0, 1], thickness=50)
    # # g0g1
    # for i, onedcurve in enumerate(onedcurves_g0g1):
    #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 1.0, 0.0, 1], thickness=10)
    # for i, onedcurve in enumerate(onedcurves_g0g1_cdfree):
    #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 1.0, 0.0, 1], thickness=50)
    # # s0s1
    # for i, onedcurve in enumerate(onedcurves_s0s1):
    #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 1.0, 1.0, 1], thickness=10)
    # for i, onedcurve in enumerate(onedcurves_s0s1_cdfree):
    #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 1.0, 1.0, 1], thickness=50)
    # # s0g1
    # for i, onedcurve in enumerate(onedcurves_s0g1):
    #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 0.0, 1], thickness=10)
    # for i, onedcurve in enumerate(onedcurves_s0g1_cdfree):
    #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 0.0, 1], thickness=50)
    # # s1g1
    # for i, onedcurve in enumerate(onedcurves_s1g1):
    #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 0.0, 0.0, 1], thickness=10)
    # for i, onedcurve in enumerate(onedcurves_s1g1_cdfree):
    #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 0.0, 0.0, 1], thickness=50)
    #
    # for i, grp_3d in enumerate(grppoint0_3d):
    #     pg.plotLinesegs(base.render, grp_3d, rgba=[1.0, 0.0, 0.0, 1], thickness=30)
    # for i, grp_3d in enumerate(grppoint1_3d):
    #     pg.plotLinesegs(base.render, grp_3d, rgba=[1.0, 0.0, 0.0, 1], thickness=30)
    # for i, sp_3d in enumerate(sppoint0_3d):
    #     pg.plotLinesegs(base.render, sp_3d, rgba=[0.0, 0.0, 1.0, 1], thickness=30)
    # for i, sp_3d in enumerate(sppoint1_3d):
    #     pg.plotLinesegs(base.render, sp_3d, rgba=[0.0, 0.0, 1.0, 1], thickness=30)

    breakingpolygon = mgc.stb.genPolygonFromConf(polygon, breakingconf)
    # lowestpolygonatbreaking = mgc.stb.genPolygonFromConf(polygon, lowestconfatbreaking)
    lowestpolygonatbreaking = breakingpolygon

    # thread.start_new_thread(mgc.plot2d, (polygon, grppoint0, grppoint1, sppoint0, sppoint1))
    # thread.start_new_thread(mgc.plot2d, (breakingpolygon, grppoint0, grppoint1, sppoint0, sppoint1, 'y'))
    # thread.start_new_thread(mgc.plot2d_list, ([currentpolygon0, lowestpolygonatbreaking, breakingpolygon,
    #                                            cobtsdict_g0[breakingconf[2]], cobtsdict_g1[breakingconf[2]],
    #                                            cobtsdict_s0[breakingconf[2]], cobtsdict_s1[breakingconf[2]]],
    #                                           grppoint0, grppoint1, sppoint0, sppoint1,
    #                                           ['r', 'y', 'y', 'g', 'g', 'g', 'g']))
    mgc.plot2d_list([currentpolygon0, lowestpolygonatbreaking, breakingpolygon,
                     cobtsdict_g0[breakingconf[2]], cobtsdict_g1[breakingconf[2]],
                     cobtsdict_s0[breakingconf[2]], cobtsdict_s1[breakingconf[2]]],
                    grppoint0, grppoint1, sppoint0, sppoint1,
                    ['r', 'y', 'y', 'g', 'g', 'k', 'k'])

    # pg.plotAxisSelf(base.render, length=1500, thickness=70)
    # base.run()

    return breakingconf[1] - startingconf[1]

# deprecated
# def computestability(polygon, grppoint0, grppoint1, sppoint0, sppoint1, scale = 10.0, steplength = 30, rotrange = 480.0):
#     """
#     compute the depth of valley
#     and the save the breaking states as images
#
#     # assumption:
#     # the function assumes the object could only escape from the exit formed by grppoint0-grppoint1
#     # grppoint0 and grppoint1 are higher than sppoint0 and sppoint1
#
#     :param polygon:
#     :param grppoint0:
#     :param grppoint1:
#     :param sppoint0:
#     :param sppoint1:
#     :return:
#     """
#
#     mgc = Mgc(rotrange, scale, steplength)
#
#     # polygoncenter = [polygon.centroid.x, polygon.centroid.y, rotrange* scale / 2.0]
#     # base = pc.World(camp=[-10000, 4000, 2500], lookatp=polygoncenter)
#
#     v_g0 = np.array([grppoint0.x, grppoint0.y])
#     v_g1 = np.array([grppoint1.x, grppoint1.y])
#     v_s0 = np.array([sppoint0.x, sppoint0.y])
#     v_s1 = np.array([sppoint1.x, sppoint1.y])
#
#     v_g0g1 = v_g1 - v_g0
#     v_g1g0 = v_g0 - v_g1
#
#     ## constraindirect_g0s0
#     if v_s0[1] >= v_g0[1]:
#         assert ("grppoint0 should be higher than sppoint0")
#     constraindirect_g0s0 = np.array([1.0, 0.0])
#     if v_g0[0] != v_s0[0]:
#         v_g0s0 = v_s0 - v_g0
#         v_g0s0_normalized = v_g0s0 / np.linalg.norm(v_g0s0)
#         constraindirect_g0s0 = v_g0 + np.dot(-v_g0, v_g0s0_normalized) * v_g0s0_normalized
#         constraindirect_g0s0 = constraindirect_g0s0 / np.linalg.norm(constraindirect_g0s0)
#         # pg.plotArrow(base.render, np.array([0,0,0]), (np.array([0,0,0])+np.array([constraindirect_g0s0[0], constraindirect_g0s0[1], 0]))*1000)
#         if rm.degree_between(constraindirect_g0s0, v_g0g1) > 90:
#             constraindirect_g0s0 = -constraindirect_g0s0
#     ## constraindirect_g0s1
#     if v_s1[1] >= v_g0[1]:
#         assert ("grppoint0 should be higher than sppoint1")
#     constraindirect_g0s1 = np.array([1.0, 0.0])
#     if v_g0[0] != v_s1[0]:
#         v_g0s1 = v_s1 - v_g0
#         v_g0s1_normalized = v_g0s1 / np.linalg.norm(v_g0s1)
#         constraindirect_g0s1 = v_g0 + np.dot(-v_g0, v_g0s1_normalized) * v_g0s1_normalized
#         constraindirect_g0s1 = constraindirect_g0s1 / np.linalg.norm(constraindirect_g0s1)
#         if rm.degree_between(constraindirect_g0s1, v_g0g1) > 90:
#             constraindirect_g0s1 = -constraindirect_g0s1
#     ## constraindirect_g0g1
#     constraindirect_g0g1 = np.array([0.0, 1.0])
#     if v_g0[1] != v_g1[1]:
#         v_g0g1 = v_g1 - v_g0
#         v_g0g1_normalized = v_g0g1 / np.linalg.norm(v_g0g1)
#         constraindirect_g0g1 = v_g0 + np.dot(-v_g0, v_g0g1_normalized) * v_g0g1_normalized
#         constraindirect_g0g1 = constraindirect_g0g1 / np.linalg.norm(constraindirect_g0g1)
#     ## constraindirect_s0s1
#     constraindirect_s0s1 = np.array([0.0, 1.0])
#     if v_s1[1] != v_s0[1]:
#         v_s0s1 = v_s1 - v_s0
#         v_s0s1_normalized = v_s0s1 / np.linalg.norm(v_s0s1)
#         constraindirect_s0s1 = v_s0 + np.dot(-v_s0, v_s0s1_normalized) * v_s0s1_normalized
#         constraindirect_s0s1 = constraindirect_s0s1 / np.linalg.norm(constraindirect_s0s1)
#     ## constraindirect_s0g1
#     if v_s0[1] >= v_g1[1]:
#         assert ("grppoint1 should be higher than sppoint0")
#     constraindirect_g1s0 = np.array([-1.0, 0.0])
#     if v_g1[0] != v_s0[0]:
#         v_s0g1 = v_g1 - v_s0
#         v_s0g1_normalized = v_s0g1 / np.linalg.norm(v_s0g1)
#         constraindirect_s0g1 = v_s0 + np.dot(-v_s0, v_s0g1_normalized) * v_s0g1_normalized
#         constraindirect_s0g1 = constraindirect_s0g1 / np.linalg.norm(constraindirect_s0g1)
#         if rm.degree_between(constraindirect_s0g1, v_g1g0) > 90:
#             constraindirect_s0g1 = -constraindirect_s0g1
#     ## constraindirect_s1g1
#     if v_s1[1] >= v_g1[1]:
#         assert ("grppoint1 should be higher than sppoint1")
#     constraindirect_g1s1 = np.array([-1.0, 0.0])
#     if v_g1[0] != v_s1[0]:
#         v_s1g1 = v_g1 - v_s1
#         v_s1g1_normalized = v_s1g1 / np.linalg.norm(v_s1g1)
#         constraindirect_s1g1 = v_s1 + np.dot(-v_s1, v_s1g1_normalized) * v_s1g1_normalized
#         constraindirect_s1g1 = constraindirect_s1g1 / np.linalg.norm(constraindirect_s1g1)
#         if rm.degree_between(constraindirect_s1g1, v_g1g0) > 90:
#             constraindirect_s1g1 = -constraindirect_s1g1
#
#     # add exit
#     # constraindirect_g0s0 = constraindirect_g0s0+constraindirect_g0g1
#     # constraindirect_g0s1 = constraindirect_g0s1+constraindirect_g0g1
#     # constraindirect_s0g1 = constraindirect_s0g1+constraindirect_g0g1
#     # constraindirect_s1g1 = constraindirect_s1g1+constraindirect_g0g1
#
#     # generate 6 curves
#     # curve 1: grppoint0-sppoint0 curves
#     [onedcurves_g0s0, polygonxoncurves_g0s0, grppoint0_3d, sppoint0_3d] = \
#         mgc.stb.genOneDCurve(polygon, grppoint0, sppoint0, constraindirect=constraindirect_g0s0)
#     # curve 2: grppoint0-sppoint1 curve
#     [onedcurves_g0s1, polygonxoncurves_g0s1, grppoint0_3d, sppoint1_3d] = \
#         mgc.stb.genOneDCurve(polygon, grppoint0, sppoint1, constraindirect=constraindirect_g0s1)
#     # curve 3: grppoint0-grppoint1 curve
#     [onedcurves_g0g1, polygonxoncurves_g0g1, grppoint0_3d, grppoint1_3d] = \
#         mgc.stb.genOneDCurve(polygon, grppoint0, grppoint1, constraindirect=constraindirect_g0g1)
#     # curve 4: sppoint0-sppoint1 curve
#     [onedcurves_s0s1, polygonxoncurves_s0s1, sppoint0_3d, sppoint1_3d] = \
#         mgc.stb.genOneDCurve(polygon, sppoint0, sppoint1, constraindirect=constraindirect_s0s1)
#     # curve 5: sppoint0-grppoint1 curve
#     [onedcurves_s0g1, polygonxoncurves_s0g1, sppoint0_3d, grppoint1_3d] = \
#         mgc.stb.genOneDCurve(polygon, sppoint0, grppoint1, constraindirect=constraindirect_s0g1)
#     # curve 6: sppoint1-grppoint1 curve
#     [onedcurves_s1g1, polygonxoncurves_s1g1, sppoint1_3d, grppoint1_3d] = \
#         mgc.stb.genOneDCurve(polygon, sppoint1, grppoint1, constraindirect=constraindirect_s1g1)
#
#     # plot cobt
#     # color of finger 0 => red
#     # mgc.plotCobt(base.render, grppoint0, polygon, rgba=[.7, .3, .3, .7])
#     # color of finger 1 => blue
#     # mgc.plotCobt(base.render, grppoint1, polygon, rgba=[.3, .3, .7, .7])
#     # color of supports => gray
#     # mgc.plotCobt(base.render, sppoint0, polygon, rgba=[.5, .5, .5, .5])
#     # mgc.plotCobt(base.render, sppoint1, polygon, rgba=[.5, .5, .5, .5])
#
#     # convert multiple curves (segmented) to a dictionary
#     onedcurvedict_g0s0 = mgc.cvtcurveList2Dict(onedcurves_g0s0)
#     onedcurvedict_g0s1 = mgc.cvtcurveList2Dict(onedcurves_g0s1)
#     onedcurvedict_g0g1 = mgc.cvtcurveList2Dict(onedcurves_g0g1)
#     onedcurvedict_s0s1 = mgc.cvtcurveList2Dict(onedcurves_s0s1)
#     onedcurvedict_s0g1 = mgc.cvtcurveList2Dict(onedcurves_s0g1)
#     onedcurvedict_s1g1 = mgc.cvtcurveList2Dict(onedcurves_s1g1)
#     # convert multiple polygons on the multiple curves to a dictionary
#     polygonxoncurvedict_g0s0 = mgc.cvtpolygonList2Dict(polygonxoncurves_g0s0, onedcurves_g0s0)
#     polygonxoncurvedict_g0s1 = mgc.cvtpolygonList2Dict(polygonxoncurves_g0s1, onedcurves_g0s1)
#     polygonxoncurvedict_g0g1 = mgc.cvtpolygonList2Dict(polygonxoncurves_g0g1, onedcurves_g0g1)
#     polygonxoncurvedict_s0s1 = mgc.cvtpolygonList2Dict(polygonxoncurves_s0s1, onedcurves_s0s1)
#     polygonxoncurvedict_s0g1 = mgc.cvtpolygonList2Dict(polygonxoncurves_s0g1, onedcurves_s0g1)
#     polygonxoncurvedict_s1g1 = mgc.cvtpolygonList2Dict(polygonxoncurves_s1g1, onedcurves_s1g1)
#
#     cobtsdict_g0 = mgc.stb.genCobt(polygon, grppoint0)
#     cobtsdict_g1 = mgc.stb.genCobt(polygon, grppoint1)
#     cobtsdict_s0 = mgc.stb.genCobt(polygon, sppoint0)
#     cobtsdict_s1 = mgc.stb.genCobt(polygon, sppoint1)
#
#     ## remove collided curves between curve_g0s0 and obt g1, obt s1
#     onedcurvedict_g0s0_rg1, polygonxoncurvedict_g0s0_rg1 = \
#         mgc.removeCollisiononOnedCurves(onedcurvedict_g0s0, polygonxoncurvedict_g0s0, cobtsdict_g1)
#     onedcurvedict_g0s0_rg1s1, polygonxoncurvedict_g0s0_rg1s1 = \
#         mgc.removeCollisiononOnedCurves(onedcurvedict_g0s0_rg1, polygonxoncurvedict_g0s0_rg1, cobtsdict_s1)
#     onedcurves_g0s0_cdfree = mgc.cvtcurveDict2List(onedcurvedict_g0s0_rg1s1)
#     polygonxoncurves_g0s0_cdfree = mgc.cvtpolygonDict2List(polygonxoncurvedict_g0s0_rg1s1)
#     ## remove collided curves between curve_g0s1 and obt g1, obt s0
#     onedcurvedict_g0s1_rg1, polygonxoncurvedict_g0s1_rg1 = \
#         mgc.removeCollisiononOnedCurves(onedcurvedict_g0s1, polygonxoncurvedict_g0s1, cobtsdict_g1)
#     onedcurvedict_g0s1_rg1s0, polygonxoncurvedict_g0s1_rg1s0 = \
#         mgc.removeCollisiononOnedCurves(onedcurvedict_g0s1_rg1, polygonxoncurvedict_g0s1_rg1, cobtsdict_s0)
#     onedcurves_g0s1_cdfree = mgc.cvtcurveDict2List(onedcurvedict_g0s1_rg1s0)
#     polygonxoncurves_g0s1_cdfree = mgc.cvtpolygonDict2List(polygonxoncurvedict_g0s1_rg1s0)
#     ## remove collided curves between curve_g0g1 and obt s0, obt s1
#     onedcurvedict_g0g1_rs0, polygonxoncurvedict_g0g1_rs0 = \
#         mgc.removeCollisiononOnedCurves(onedcurvedict_g0g1, polygonxoncurvedict_g0g1, cobtsdict_s0)
#     onedcurvedict_g0g1_rs0s1, polygonxoncurvedict_g0g1_rs0s1 = \
#         mgc.removeCollisiononOnedCurves(onedcurvedict_g0g1_rs0, polygonxoncurvedict_g0g1_rs0, cobtsdict_s1)
#     onedcurves_g0g1_cdfree = mgc.cvtcurveDict2List(onedcurvedict_g0g1_rs0s1)
#     polygonxoncurves_g0g1_cdfree = mgc.cvtpolygonDict2List(polygonxoncurvedict_g0g1_rs0s1)
#     # remove collided curves between curve_s0s1 and obt g0, obt g1
#     onedcurvedict_s0s1_rg0, polygonxoncurvedict_s0s1_rg0 = \
#         mgc.removeCollisiononOnedCurves(onedcurvedict_s0s1, polygonxoncurvedict_s0s1, cobtsdict_g0)
#     onedcurvedict_s0s1_rg0g1, polygonxoncurvedict_s0s1_rg0g1 = \
#         mgc.removeCollisiononOnedCurves(onedcurvedict_s0s1_rg0, polygonxoncurvedict_s0s1_rg0, cobtsdict_g1)
#     onedcurves_s0s1_cdfree = mgc.cvtcurveDict2List(onedcurvedict_s0s1_rg0g1)
#     polygonxoncurves_s0s1_cdfree = mgc.cvtpolygonDict2List(polygonxoncurvedict_s0s1_rg0g1)
#     ## remove collided curves between curve_s0g1 and obt s1, obt g0
#     onedcurvedict_s0g1_rs1, polygonxoncurvedict_s0g1_rs1 = \
#         mgc.removeCollisiononOnedCurves(onedcurvedict_s0g1, polygonxoncurvedict_s0g1, cobtsdict_s1)
#     onedcurvedict_s0g1_rs1g0, polygonxoncurvedict_s0g1_rs1g0 = \
#         mgc.removeCollisiononOnedCurves(onedcurvedict_s0g1_rs1, polygonxoncurvedict_s0g1_rs1, cobtsdict_g0)
#     onedcurves_s0g1_cdfree = mgc.cvtcurveDict2List(onedcurvedict_s0g1_rs1g0)
#     polygonxoncurves_s0g1_cdfree = mgc.cvtpolygonDict2List(polygonxoncurvedict_s0g1_rs1g0)
#     ## remove collided curves between curve_s1g1 and obt g0, obt s0
#     onedcurvedict_s1g1_rg0, polygonxoncurvedict_s1g1_rg0 = \
#         mgc.removeCollisiononOnedCurves(onedcurvedict_s1g1, polygonxoncurvedict_s1g1, cobtsdict_g0)
#     onedcurvedict_s1g1_rg0s0, polygonxoncurvedict_s1g1_rg0s0 = \
#         mgc.removeCollisiononOnedCurves(onedcurvedict_s1g1_rg0, polygonxoncurvedict_s1g1_rg0, cobtsdict_s0)
#     onedcurves_s1g1_cdfree = mgc.cvtcurveDict2List(onedcurvedict_s1g1_rg0s0)
#     polygonxoncurves_s1g1_cdfree = mgc.cvtpolygonDict2List(polygonxoncurvedict_s1g1_rg0s0)
#
#     # plot maxima
#     # g0s0
#     # for i, onedcurve in en
#     # ity[2]], radius=100, rgba=[1, 1, 0, .5])
#
#     # startingconf = []
#     # breakingconf = []
#     # lowestconfatbreaking = []
#     # the initial configuration is assumed to be half of heighrange
#     currentkey = 0
#     currentpolygon0 = []
#     for i, polygonxoncurve in enumerate(polygonxoncurves_s0s1):
#         for j, polygonx in enumerate(polygonxoncurve):
#             if math.fabs(onedcurves_s0s1[i][j][2] - mgc.heightrange / 2.0) < steplength:
#                 currentkey = onedcurves_s0s1[i][j][2]
#                 currentpolygon0 = polygonx
#                 currentconf = onedcurves_s0s1[i][j]
#                 # pg.plotSphere(base.render, onedcurves_s0s1[i][j], radius=300, rgba=[1, 0, 0, 1])
#                 # height = onedcurves_s0s1[i][j][2]
#                 # polygonsnp = mgc.stb.genPolygonsnp(polygonx, height, color=[0.0, 0.5, 0.0, 1], thickness=20)
#                 # for polygonnp in polygonsnp:
#                 #     polygonnp.reparentTo(base.render)
#     curvepoint = onedcurvedict_s0s1[currentkey]
#     polylist_onecobtslice = [cobtsdict_g0[currentkey], cobtsdict_s0[currentkey],
#                              cobtsdict_s1[currentkey], cobtsdict_g1[currentkey]]
#     othercurvepoints = []
#     a = onedcurvedict_g0s0_rg1s1[currentkey]
#     b = onedcurvedict_g0s1_rg1s0[currentkey]
#     c = onedcurvedict_g0g1_rs0s1[currentkey]
#     d = onedcurvedict_s0s1_rg0g1[currentkey]
#     e = onedcurvedict_s0g1_rs1g0[currentkey]
#     f = onedcurvedict_s1g1_rg0s0[currentkey]
#     if a[1] > mgc.minusinf + 1:
#         othercurvepoints.append([a[0], a[1], currentkey])
#     if b[1] > mgc.minusinf + 1:
#         othercurvepoints.append([b[0], b[1], currentkey])
#     if c[1] > mgc.minusinf + 1:
#         othercurvepoints.append([c[0], c[1], currentkey])
#     if d[1] > mgc.minusinf + 1:
#         othercurvepoints.append([d[0], d[1], currentkey])
#     if e[1] > mgc.minusinf + 1:
#         othercurvepoints.append([e[0], e[1], currentkey])
#     if f[1] > mgc.minusinf + 1:
#         othercurvepoints.append([f[0], f[1], currentkey])
#     breakingmaxima, breakingconfiguration, valleyminima, bottomconfiguration = \
#         mgc.lstb.maxmin_oneslice(curvepoint, polylist_onecobtslice, othercurvepoints)
#     # pg.plotSphere(base.render, breakingconfiguration, radius=200, rgba=[.3, 0, .3, 1])
#     # pg.plotSphere(base.render, bottomconfiguration, radius=300, rgba=[1, 0, 0, 1])
#     startingconf = bottomconfiguration
#     breakingconf = breakingconfiguration
#     lowestconfatbreaking = bottomconfiguration
#
#     lowestonedcurve = {}
#     for key in range(0, mgc.heightrange, mgc.steplength):
#         vert = [mgc.minusinf, mgc.minusinf, key]
#         a = onedcurvedict_g0s0_rg1s1[key]
#         b = onedcurvedict_g0s1_rg1s0[key]
#         c = onedcurvedict_g0g1_rs0s1[key]
#         d = onedcurvedict_s0s1_rg0g1[key]
#         e = onedcurvedict_s0g1_rs1g0[key]
#         f = onedcurvedict_s1g1_rg0s0[key]
#         # print a,b,c,d,e,f
#         # time.sleep(1.0)
#         vertlist = []
#         if a[1] > mgc.minusinf + 1:
#             vertlist.append([a[0], a[1], key])
#         if b[1] > mgc.minusinf + 1:
#             vertlist.append([b[0], b[1], key])
#         if c[1] > mgc.minusinf + 1:
#             vertlist.append([c[0], c[1], key])
#         if d[1] > mgc.minusinf + 1:
#             vertlist.append([d[0], d[1], key])
#         if e[1] > mgc.minusinf + 1:
#             vertlist.append([e[0], e[1], key])
#         if f[1] > mgc.minusinf + 1:
#             vertlist.append([f[0], f[1], key])
#         if len(vertlist) > 0:
#             vert = min(vertlist, key=lambda item: item[1])
#             # print vert
#         lowestonedcurve[key] = vert
#     # # minus 180
#     for i in range(-1, -int(180 * scale / float(steplength)), -1):
#         nxtangle = i * steplength
#         key = currentkey + nxtangle
#         curvepoint = lowestonedcurve[key]
#         if curvepoint[1] > mgc.minusinf+1:
#             polylist_onecobtslice = [cobtsdict_g0[key], cobtsdict_s0[key],
#                                      cobtsdict_s1[key], cobtsdict_g1[key]]
#             othercurvepoints = []
#             a = onedcurvedict_g0s0_rg1s1[key]
#             b = onedcurvedict_g0s1_rg1s0[key]
#             c = onedcurvedict_g0g1_rs0s1[key]
#             d = onedcurvedict_s0s1_rg0g1[key]
#             e = onedcurvedict_s0g1_rs1g0[key]
#             f = onedcurvedict_s1g1_rg0s0[key]
#             if a[1] > mgc.minusinf + 1:
#                 othercurvepoints.append([a[0], a[1], key])
#             if b[1] > mgc.minusinf + 1:
#                 othercurvepoints.append([b[0], b[1], key])
#             if c[1] > mgc.minusinf + 1:
#                 othercurvepoints.append([c[0], c[1], key])
#             if d[1] > mgc.minusinf + 1:
#                 othercurvepoints.append([d[0], d[1], key])
#             if e[1] > mgc.minusinf + 1:
#                 othercurvepoints.append([e[0], e[1], key])
#             if f[1] > mgc.minusinf + 1:
#                 othercurvepoints.append([f[0], f[1], key])
#             breakingmaxima, breakingconfiguration, valleyminima, bottomconfiguration = \
#                 mgc.lstb.maxmin_oneslice(curvepoint, polylist_onecobtslice, othercurvepoints)
#             # pg.plotSphere(base.render, breakingconfiguration, radius=200, rgba=[.3, 0, .3, 1])
#             # pg.plotSphere(base.render, bottomconfiguration, radius=200, rgba=[.3, 0, 0, 1])
#             if breakingconfiguration[1] < breakingconf[1]:
#                 breakingconf = breakingconfiguration
#                 lowestconfatbreaking = bottomconfiguration
#     # plus 180
#     for i in range(1, int(180 * scale / float(steplength))):
#         nxtangle = i * steplength
#         key = currentkey + nxtangle
#         curvepoint = lowestonedcurve[key]
#         if curvepoint[1] > mgc.minusinf+1:
#             polylist_onecobtslice = [cobtsdict_g0[key], cobtsdict_s0[key],
#                                      cobtsdict_s1[key], cobtsdict_g1[key]]
#             othercurvepoints = []
#             a = onedcurvedict_g0s0_rg1s1[key]
#             b = onedcurvedict_g0s1_rg1s0[key]
#             c = onedcurvedict_g0g1_rs0s1[key]
#             d = onedcurvedict_s0s1_rg0g1[key]
#             e = onedcurvedict_s0g1_rs1g0[key]
#             f = onedcurvedict_s1g1_rg0s0[key]
#             if a[1] > mgc.minusinf + 1:
#                 othercurvepoints.append([a[0], a[1], key])
#             if b[1] > mgc.minusinf + 1:
#                 othercurvepoints.append([b[0], b[1], key])
#             if c[1] > mgc.minusinf + 1:
#                 othercurvepoints.append([c[0], c[1], key])
#             if d[1] > mgc.minusinf + 1:
#                 othercurvepoints.append([d[0], d[1], key])
#             if e[1] > mgc.minusinf + 1:
#                 othercurvepoints.append([e[0], e[1], key])
#             if f[1] > mgc.minusinf + 1:
#                 othercurvepoints.append([f[0], f[1], key])
#             breakingmaxima, breakingconfiguration, valleyminima, bottomconfiguration = \
#                 mgc.lstb.maxmin_oneslice(curvepoint, polylist_onecobtslice, othercurvepoints)
#             # pg.plotSphere(base.render, breakingconfiguration, radius=200, rgba=[.3, 0, .3, 1])
#             # pg.plotSphere(base.render, bottomconfiguration, radius=200, rgba=[.3, 0, 0, 1])
#             if breakingconfiguration[1] < breakingconf[1]:
#                 breakingconf = breakingconfiguration
#                 lowestconfatbreaking = bottomconfiguration
#
#     # global breaking conf
#     # pg.plotSphere(base.render, breakingconf, radius=500, rgba=[1, 1, 0, 1])
#
#     # # g0s0
#     # for i, onedcurve in enumerate(onedcurves_g0s0):
#     #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 1.0, 1], thickness=10)
#     # for i, onedcurve in enumerate(onedcurves_g0s0_cdfree):
#     #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 1.0, 1], thickness=50)
#     # # pg.plotArrow(base.render, spos = np.array([0,0,0]), epos=np.array([0,0,0])+np.array([constraindirect_g0s0[0], constraindirect_g0s0[1], 0]), length = 10000)
#     # # g0s1
#     # for i, onedcurve in enumerate(onedcurves_g0s1):
#     #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 0.0, 1.0, 1], thickness=10)
#     # for i, onedcurve in enumerate(onedcurves_g0s1_cdfree):
#     #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 0.0, 1.0, 1], thickness=50)
#     # # g0g1
#     # for i, onedcurve in enumerate(onedcurves_g0g1):
#     #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 1.0, 0.0, 1], thickness=10)
#     # for i, onedcurve in enumerate(onedcurves_g0g1_cdfree):
#     #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 1.0, 0.0, 1], thickness=50)
#     # # s0s1
#     # for i, onedcurve in enumerate(onedcurves_s0s1):
#     #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 1.0, 1.0, 1], thickness=10)
#     # for i, onedcurve in enumerate(onedcurves_s0s1_cdfree):
#     #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 1.0, 1.0, 1], thickness=50)
#     # # s0g1
#     # for i, onedcurve in enumerate(onedcurves_s0g1):
#     #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 0.0, 1], thickness=10)
#     # for i, onedcurve in enumerate(onedcurves_s0g1_cdfree):
#     #     pg.plotLinesegs(base.render, onedcurve, rgba=[1.0, 1.0, 0.0, 1], thickness=50)
#     # # s1g1
#     # for i, onedcurve in enumerate(onedcurves_s1g1):
#     #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 0.0, 0.0, 1], thickness=10)
#     # for i, onedcurve in enumerate(onedcurves_s1g1_cdfree):
#     #     pg.plotLinesegs(base.render, onedcurve, rgba=[0.0, 0.0, 0.0, 1], thickness=50)
#
#     # for i, grp_3d in enumerate(grppoint0_3d):
#     #     pg.plotLinesegs(base.render, grp_3d, rgba=[1.0, 0.0, 0.0, 1], thickness=30)
#     # for i, grp_3d in enumerate(grppoint1_3d):
#     #     pg.plotLinesegs(base.render, grp_3d, rgba=[1.0, 0.0, 0.0, 1], thickness=30)
#     # for i, sp_3d in enumerate(sppoint0_3d):
#     #     pg.plotLinesegs(base.render, sp_3d, rgba=[0.0, 0.0, 1.0, 1], thickness=30)
#     # for i, sp_3d in enumerate(sppoint1_3d):
#     #     pg.plotLinesegs(base.render, sp_3d, rgba=[0.0, 0.0, 1.0, 1], thickness=30)
#
#     breakingpolygon = mgc.stb.genPolygonFromConf(polygon, breakingconf)
#     lowestpolygonatbreaking = mgc.stb.genPolygonFromConf(polygon, lowestconfatbreaking)
#
#     # thread.start_new_thread(mgc.plot2d, (polygon, grppoint0, grppoint1, sppoint0, sppoint1))
#     # thread.start_new_thread(mgc.plot2d, (breakingpolygon, grppoint0, grppoint1, sppoint0, sppoint1, 'y'))
#     # thread.start_new_thread(mgc.plot2d_list, ([currentpolygon0, lowestpolygonatbreaking, breakingpolygon,
#     #                                            cobtsdict_g0[breakingconf[2]], cobtsdict_g1[breakingconf[2]],
#     #                                            cobtsdict_s0[breakingconf[2]], cobtsdict_s1[breakingconf[2]]],
#     #                                           grppoint0, grppoint1, sppoint0, sppoint1,
#     #                                           ['r', 'y', 'y', 'g', 'g', 'g', 'g']))
#     mgc.plot2d_list([currentpolygon0, lowestpolygonatbreaking, breakingpolygon,
#                      cobtsdict_g0[breakingconf[2]], cobtsdict_g1[breakingconf[2]],
#                      cobtsdict_s0[breakingconf[2]], cobtsdict_s1[breakingconf[2]]],
#                     grppoint0, grppoint1, sppoint0, sppoint1,
#                     ['r', 'c', 'y', 'g', 'k', 'k', 'g'])
#
#     # pg.plotAxisSelf(base.render, length=1500, thickness=70)
#     # base.run()
#
#     return breakingconf[1] - startingconf[1]

if __name__== "__main__":
    polygon = Polygon([[0.0 ,0.0],
                       [1000.0 ,0.0],
                       [2000.0 ,1000.0],
                       [1000.0 ,1500.0],
                       [0.0 ,1000.0]])

    # plot
    scale = 10.0
    steplength = 30
    rotrange = 480.0


    grppoints = {}

    # exp2
    # grp0 = np.array([-700.0, 1120.0])
    # grp1 = np.array([1440.0, 820.0])

    # exp3
    grp0 = np.array([-800.0, 1250.0])
    grp1 = np.array([1440.0, 520.0])

    vec = grp1 - grp0
    veclength = np.linalg.norm(vec)
    vecdirect = vec / veclength

    vecstep = veclength / 40.0
    for i in range(0, 8):
        grppoint0 = vecstep * i * vecdirect / 2.0 + grp0
        grppoint1 = -vecstep * i * vecdirect / 2.0 + grp1
        grppoints[i] = [Point(grppoint0[0], grppoint0[1]), Point(grppoint1[0], grppoint1[1])]

    sppoint0 = Point(0.0, 100.0)
    sppoint1 = Point(900.0, 0.0)

    depth = []
    for i in range(0,8):
        depth.append(computestability(polygon, grppoints[i][0], grppoints[i][1],
                                      sppoint0, sppoint1, scale=10.0, steplength=30, rotrangle=480.0))
        print depth
        # print grppoints[i][0], grppoints[i][1]

    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.plot(range(0,8), depth, 'r-')
    fig.savefig(str(grp0+grp1)+'.png')