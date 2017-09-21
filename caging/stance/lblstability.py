import os
import math
from panda3d.core import *
import pandaplotutils.pandageom as pg
import pandaplotutils.pandactrl as pc
from shapely.geometry import Polygon
from shapely.geometry import Point
from trimesh.transformations import *
from shapely import affinity
import numpy as np

class Lblstability(object):
    """
    compute the stability depth layer by layer
    author: weiwei
    date: 20170823
    """

    def __init__(self, rotrange, scale, steplength):
        self.rotrange = rotrange
        self.scale = scale
        self.steplength = steplength
        self.heightrange = int(self.rotrange*self.scale)
        self.minusinf = -999999999999
        self.plusinf = 999999999999
        self.distthreshold = 20.0

    def boundarymaxima(self, curvepoint_shapely, boundary_shapely, othercurvepoints, tag = "idxplus"):
        """
        compute the maxima of boundary_shapely on the tag direction,
        curvepoint_shapely is also counted during comparison

        :param curvepoint_shapely:
        :param boundary_shapely:
        :param tag:
        :return:

        author: weiwei
        date: 20170823
        """

        verts = list(boundary_shapely.coords)
        index = -1
        mindist = self.plusinf
        for i in range(0, len(verts)):
            dist = Point(verts[i][0], verts[i][1]).distance(curvepoint_shapely)
            if dist < mindist:
                mindist = dist
                index = i
        value = [curvepoint_shapely.x, curvepoint_shapely.y]
        if tag == "idxplus":
            idxplus = index
            # while verts[idxplus][0] >= curvepoint_shapely.x:
            while True:
                idxplus = (idxplus+1)%len(verts)
                if verts[idxplus][1] > value[1]-self.distthreshold:
                    value = verts[idxplus][:2]
                # if going down
                if verts[idxplus][1] < curvepoint_shapely.y and \
                                math.fabs(verts[idxplus][1]-curvepoint_shapely.y) > self.distthreshold:
                    bcont = False
                    for othercp in othercurvepoints:
                        if Point(othercp[0], othercp[1]).distance(Point(value[0], value[1])) < 1.0:
                            bcont = True
                            break
                    if not bcont:
                        break
        elif tag == "idxminus":
            idxminus = index
            # while verts[idxminus][0] <= curvepoint_shapely.x:
            while True:
                idxminus = (idxminus-1)%len(verts)
                if verts[idxminus][1] > value[1]-self.distthreshold:
                    value = verts[idxminus][:2]
                # if going down
                if verts[idxminus][1] < curvepoint_shapely.y and \
                                math.fabs(verts[idxminus][1]-curvepoint_shapely.y) > self.distthreshold:
                    bcont = False
                    for othercp in othercurvepoints:
                        if Point(othercp[0], othercp[1]).distance(Point(value[0], value[1])) < 1.0:
                            bcont = True
                            break
                    if not bcont:
                        break
        else:
            assert("Wrong tag! Must be xplus or xminus!")
        return Point(value[0], value[1])

    def boundaryminima(self, curvepoint_shapely, maxima, boundary_shapely):
        """
        compute the minima of boundary_shapely between curvepoint_shapely and maxima
        both curvepoint_shapely and maxima are included

        :param curvepoint_shapely:
        :param boundary_shapely:
        :param tag:
        :return:

        author: weiwei
        date: 20170823
        """

        # simplify the computation
        return curvepoint_shapely

        # verts = list(boundary_shapely.coords)
        # halfverts = []
        # if maxima.x > curvepoint_shapely.x:
        #     halfverts = [vert for vert in verts if vert[0] >= curvepoint_shapely.x and vert[0] <= maxima.x]
        # elif maxima.x < curvepoint_shapely.x:
        #     halfverts = [vert for vert in verts if vert[0] <= curvepoint_shapely.x and vert[0] <= maxima.x]
        # elif maxima.x == curvepoint_shapely.x:
        #     return curvepoint_shapely
        # else:
        #     assert("Wrong tag! Must be xplus or xminus!")
        # value = min(halfverts, key = lambda item: item[1])
        # return Point(value[0], value[1])

    def maxmin_oneslice(self, curvepoint, polylist_onecobtslice, othercurvepoints):
        """
        extract the boundary touching curvepoint using the cobtslices
        at a specific orientation

        :param curvepoint:
        :param cobtslicelist:
        :return:
        """

        curvepoint_shapely = Point(curvepoint[0], curvepoint[1])
        cobtsliceunion = polylist_onecobtslice[0]
        for i in range(1, len(polylist_onecobtslice)):
            cobtsliceunion = cobtsliceunion.union(polylist_onecobtslice[i])

        if cobtsliceunion.geom_type == 'MultiPolygon':
            mindist = self.plusinf
            for poly in cobtsliceunion:
                if poly.contains(curvepoint_shapely):
                    resultingpoly = poly
                    break
                dist = curvepoint_shapely.distance(poly.exterior)
                if dist < mindist:
                    mindist = dist
                    resultingpoly = poly
        elif cobtsliceunion.geom_type == "Polygon":
            resultingpoly = cobtsliceunion

        boundary = resultingpoly.exterior
        binterior = False
        if len(list(resultingpoly.interiors)) != 0:
            for bd in resultingpoly.interiors:
                if bd.distance(curvepoint_shapely) < boundary.distance(curvepoint_shapely):
                    binterior = True
                    break

        breakingmaxima = self.plusinf
        breakingconfiguration = [self.plusinf, self.plusinf, curvepoint[2]]
        valleyminima = curvepoint_shapely.y
        bottomconfiguration = [curvepoint_shapely.x, curvepoint_shapely.y, curvepoint[2]]

        if not binterior:
            maxima_xplus = self.boundarymaxima(curvepoint_shapely, boundary, othercurvepoints, tag = "idxplus")
            maxima_xminus = self.boundarymaxima(curvepoint_shapely, boundary, othercurvepoints, tag = "idxminus")
            minima_xplus = self.boundaryminima(curvepoint_shapely, maxima_xplus, boundary)
            minima_xminus= self.boundaryminima(curvepoint_shapely, maxima_xminus, boundary)
            if maxima_xplus.y <= breakingmaxima:
                breakingmaxima = maxima_xplus.y
                breakingconfiguration[0] = maxima_xplus.x
                breakingconfiguration[1] = maxima_xplus.y
            if maxima_xminus.y <= breakingmaxima:
                breakingmaxima = maxima_xminus.y
                breakingconfiguration[0] = maxima_xminus.x
                breakingconfiguration[1] = maxima_xminus.y
            if minima_xplus.y <= valleyminima:
                valleyminima = minima_xplus.y
                bottomconfiguration[0] = minima_xplus.x
                bottomconfiguration[1] = minima_xplus.y
            if minima_xminus.y <= valleyminima:
                valleyminima = minima_xminus.y
                bottomconfiguration[0] = minima_xplus.x
                bottomconfiguration[1] = minima_xplus.y
            # pg.plotSphere(base.render, [maxima_xplus.x, maxima_xplus.y, curvepoint[2]], radius=500, rgba=[1, 1, 1, 1])
            # pg.plotSphere(base.render, [maxima_xminus.x, maxima_xminus.y, curvepoint[2]], radius=500, rgba=[1, 1, 1, 1])
        else:
            curvepoint_shapely = Point(0,0)
            mindist = self.plusinf
            minheight = self.plusinf
            for othercurvepoint in othercurvepoints:
                # pg.plotSphere(base.render, [othercurvepoint[0], othercurvepoint[1], curvepoint[2]], radius=100, rgba=[1, 1, 1, 1])
                curvepoint_shapely_tmp = Point(othercurvepoint[0], othercurvepoint[1])
                dist = boundary.distance(curvepoint_shapely_tmp)
                # print dist, othercurvepoint
                if dist < 1:
                    if curvepoint_shapely_tmp.y < minheight:
                        minheight = curvepoint_shapely_tmp.y
                        curvepoint_shapely = Point(curvepoint_shapely_tmp.x, curvepoint_shapely_tmp.y)
            bottomconfiguration = [curvepoint_shapely.x, curvepoint_shapely.y, curvepoint[2]]
            if boundary.distance(curvepoint_shapely) < self.distthreshold:
                maxima_xplus = self.boundarymaxima(curvepoint_shapely, boundary, othercurvepoints, tag = "idxplus")
                maxima_xminus = self.boundarymaxima(curvepoint_shapely, boundary, othercurvepoints, tag = "idxminus")
                minima_xplus = self.boundaryminima(curvepoint_shapely, maxima_xplus, boundary)
                minima_xminus= self.boundaryminima(curvepoint_shapely, maxima_xminus, boundary)
                if maxima_xplus.y <= breakingmaxima:
                    breakingmaxima = maxima_xplus.y
                    breakingconfiguration[0] = maxima_xplus.x
                    breakingconfiguration[1] = maxima_xplus.y
                if maxima_xminus.y <= breakingmaxima:
                    breakingmaxima = maxima_xminus.y
                    breakingconfiguration[0] = maxima_xminus.x
                    breakingconfiguration[1] = maxima_xminus.y
                if minima_xplus.y <= valleyminima:
                    valleyminima = minima_xplus.y
                    bottomconfiguration[0] = minima_xplus.x
                    bottomconfiguration[1] = minima_xplus.y
                if minima_xminus.y <= valleyminima:
                    valleyminima = minima_xminus.y
                    bottomconfiguration[0] = minima_xplus.x
                    bottomconfiguration[1] = minima_xplus.y
            # pg.plotSphere(base.render, [curvepoint_shapely.x, curvepoint_shapely.y, curvepoint[2]], radius=200, rgba=[0, 0, 0, 1])
            # pg.plotSphere(base.render, [maxima_xplus.x, maxima_xplus.y, curvepoint[2]], radius=200, rgba=[1, 1, 1, 1])
            # pg.plotSphere(base.render, [minima_xplus.x, minima_xplus.y, curvepoint[2]], radius=200, rgba=[1, 1, 1, 1])

        return [breakingmaxima, breakingconfiguration, valleyminima, bottomconfiguration]