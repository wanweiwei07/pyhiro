#!/usr/bin/python

from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LineString
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

from caging.mergecurves import Mergecurves as Mgc
import caging.stabilityinterface as stbinter
from caging.stance.stability import Stability as Stb


if __name__== "__main__":

    """
    This function uses stabilityinterface to compute monofunctions for grips
    from different orientations. It first finds the available max and min distances between
    fingers giving an orientation between (-30, 30), and then does optimization.
    The position of fingercenter is at the upper side of object centroid.
    The exact value is 1/3 of the distance between centroid and topvertex.
    """

    polygon = Polygon([[0.0 ,0.0],
                       [1000.0 ,0.0],
                       [2000.0 ,1000.0],
                       [1000.0 ,1500.0],
                       [0.0 ,1000.0]])

    # plot
    scale = 10.0
    steplength = 30
    rotrange = 480.0

    sppoint0 = Point(0.0, 100.0)
    sppoint1 = Point(900.0, 0.0)

    skey, polygoninit  = stbinter.computeinitialpose(polygon, sppoint0, sppoint1,
                                              scale = scale, steplength = steplength, rotrange = rotrange)
    # print skey
    # print polygoninit
    polygonbd = list(polygoninit.exterior.coords)
    maxvert = max(polygonbd, key = lambda item: item[1])
    topvertex = Point(maxvert[0], maxvert[1])
    disttc = topvertex.distance(polygoninit.centroid)
    vectc = np.array([topvertex.x-polygoninit.centroid.x, topvertex.y-polygoninit.centroid.y])
    vectc = vectc/np.linalg.norm(vectc)
    fingercenter = np.array([polygoninit.centroid.x, polygoninit.centroid.y]) + vectc*1/3.0*disttc

    # rgt lowest intersections
    plusinf = 99999999999
    vec_s0 = np.array([sppoint0.x, sppoint0.y])
    vec_s1 = np.array([sppoint1.x, sppoint1.y])
    vec_s0s1 = vec_s1-vec_s0
    vec_s0s1 = vec_s0s1/np.linalg.norm(vec_s0s1)
    splinergt = vec_s0+vec_s0s1*plusinf
    splinelft = vec_s1-vec_s0s1*plusinf
    spline = LineString(([splinelft[0], splinelft[1]], [splinergt[0], splinergt[1]]))

    vecrgt = np.array([math.cos(math.radians(-30)), math.sin(math.radians(-30))])
    linp0 = fingercenter
    linp1 = fingercenter+vecrgt*plusinf
    line = LineString(([linp0[0], linp0[1]], [linp1[0], linp1[1]]))
    interpl_rgt = line.intersection(polygoninit.exterior)
    intersp_rgt = line.intersection(spline)

    # lft lowest intersections
    vecrgt = np.array([math.cos(math.radians(30)), math.sin(math.radians(30))])
    linp0 = fingercenter
    linp1 = fingercenter-vecrgt*plusinf
    line = LineString(([linp0[0], linp0[1]], [linp1[0], linp1[1]]))
    interpl_lft = line.intersection(polygoninit.exterior)
    intersp_lft = line.intersection(spline)

    minrgt = interpl_rgt.distance(polygoninit.centroid)+10.0
    maxrgt = intersp_rgt.distance(polygoninit.centroid)-10.0
    minlft = interpl_lft.distance(polygoninit.centroid)+10.0
    maxlft = intersp_lft.distance(polygoninit.centroid)-10.0
    minhalffgrdist = max([minrgt, minlft])
    maxhalffgrdist = min([maxrgt, maxlft])
    halfwidth = maxhalffgrdist - minhalffgrdist
    fgrsteplength = halfwidth/9.0

    for angle in range(-30,31,10):
        print 'angle '+str(angle)
        grppoints = {}
        vecrgt = np.array([math.cos(math.radians(angle)), math.sin(math.radians(angle))])
        grp0 = fingercenter-vecrgt*maxhalffgrdist
        grp1 = fingercenter+vecrgt*maxhalffgrdist
        for i in range(0, 8):
            grppoint0 = fgrsteplength * i * vecrgt + grp0
            grppoint1 = -fgrsteplength * i * vecrgt + grp1
            grppoints[i] = [Point(grppoint0[0], grppoint0[1]), Point(grppoint1[0], grppoint1[1])]

        depth = []
        for i in range(0,8):
            print grppoints[i][0], grppoints[i][1]
            depth.append(stbinter.computestabilitywithsk(polygon, grppoints[i][0], grppoints[i][1],
                                                   sppoint0, sppoint1, startingkey = skey, scale=scale,
                                                   steplength=steplength, rotrange=rotrange))
            print depth[i]
            # print grppoints[i][0], grppoints[i][1]

        fig = plt.figure()
        ax = fig.add_subplot(111)
        plt.plot(range(0,8), depth, 'r-')
        fig.savefig(str(grp0+grp1)+str(angle)+'.png')