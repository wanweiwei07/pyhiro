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
import time

class Stability(object):

    def __init__(self, rotrange, scale, steplength):
        self.rotrange = rotrange
        self.scale = scale
        self.heightrange = int(self.rotrange*self.scale)
        self.steplength = steplength

    def genMinkovSeg(self, polygon, segment):
        """
        convert to minkov sum, sppoint is the supporting point.
        the reference point is chosen as the geometric center of polygon

        :param polygon: polygon of the object
        :param segment: a segment finger, composed of two points [sppoint0, sppoint1]
        :param sppoint:
        :return:
        """

        resultpolpoints = []
        for i in range(2):
            sppoint = segment[i]
            if i == 0:
                vecsp = np.array([segment[1].x-segment[0].x, segment[1].y-segment[0].y,0])
            else:
                vecsp = np.array([segment[0].x-segment[1].x, segment[0].y-segment[1].y,0])
            pol = self.genMinkovPnt(polygon, sppoint)
            vertspol = pol.exterior.coords
            nvertspol = len(vertspol)
            # save data
            polysegstartid = -1
            polysegendid = -1
            for i in range(nvertspol):
                ithis = i
                inext = i+1
                inextnext = i+2
                if inext == nvertspol:
                    inext = 0
                    inextnext = 1
                if inext == nvertspol-1:
                    inextnext = 0
                vecpolnt = np.array([vertspol[inext][0]-vertspol[ithis][0],
                                    vertspol[inext][1]-vertspol[ithis][1],0])
                veccrossnext =  np.cross(vecpolnt, vecsp)
                vecpolnnt = np.array([vertspol[inextnext][0]-vertspol[inext][0],
                                    vertspol[inextnext][1]-vertspol[inext][1],0])
                veccrossnextnext =  np.cross(vecpolnnt, vecsp)
                if veccrossnext[2] <= 0 and veccrossnextnext[2] > 0 and polysegstartid == -1:
                    polysegstartid = inext
                if veccrossnext[2] > 0 and veccrossnextnext[2] <= 0 and polysegendid == -1:
                    polysegendid = inext
            if polysegendid < polysegstartid:
                resultpolpoints.extend(vertspol[polysegstartid:])
                resultpolpoints.extend(vertspol[1:polysegendid+1])
            else:
                resultpolpoints.extend(vertspol[polysegstartid:polysegendid+1])

        return Polygon(resultpolpoints)

    def genMinkovPnt(self, polygon, sppoint):
        """
        convert to minkov sum, sppoint is the supporting point.
        the reference point is chosen as the geometric center of polygon

        :param polygon:
        :param sppoint:
        :return:
        """

        return self.translate(self.rotate(polygon, 180), sppoint)

    def rotate(self, polygon, angle, refpoint=Point()):
        """
        rotate polygon around ref point

        :param polygon:
        :param angle: in degree
        :param point:
        :return:
        """

        if refpoint.is_empty:
            refpoint = polygon.centroid
            return affinity.rotate(polygon, angle, refpoint)
        else:
            return affinity.rotate(polygon, angle, refpoint)

    def translate(self, polygon, newcenterpoint):
        """
        translate polygon around ref point

        :param polygon:
        :param newcenterpoint: shapely point
        :return:
        """

        xoff = newcenterpoint.x - polygon.centroid.x
        yoff = newcenterpoint.y - polygon.centroid.y
        return affinity.translate(polygon, xoff, yoff)

    def genPolygonFromConf(self, polygon, configuration, refpoint=Point()):
        """
        recover the workspace polygon using a configuration point

        :param configuration:
        :return:
        """

        if refpoint.is_empty:
            xoff = configuration[0]
            yoff = configuration[1]
            rotpoly = self.rotate(polygon, (configuration[2])/self.scale)
            transpoly = self.translate(rotpoly, Point(xoff, yoff))
            # print configuration[0], configuration[1]
            # print transpoly.centroid.x, transpoly.centroid.y
            return transpoly
        else:
            xoff = configuration[0]
            yoff = configuration[1]
            rotpoly = self.rotate(polygon, configuration[2]/self.scale)
            transpoly = self.translate(rotpoly, Point(xoff, yoff))
            return transpoly

    def genPolygonsnp(self, polygon, height, color = [], thickness = 20.0):
        polygons = []
        if polygon.geom_type == "Polygon":
            verts3d = []
            for vert in polygon.exterior.coords:
                verts3d.append([vert[0], vert[1], height])
            polygons.append(pg.genPolygonsnp(verts3d, color, thickness))
        else:
            for polygonpart in polygon:
                verts3d = []
                for vert in polygonpart.exterior.coords:
                    verts3d.append([vert[0], vert[1], height])
            polygons.append(pg.genPolygonsnp(verts3d, color, thickness))
        return polygons

    def genCobtnp(self, polygonlist, steplength):
        """
        generate a mesh model for polygonlist

        :param polygonlist:
        :param color:
        :return:

        author: weiwei
        date: 20170517
        """

        npolygon = len(polygonlist)
        vertices = []
        facenormals = []
        triangles = []
        nvertperpolygon = len(polygonlist[0].exterior.coords)-1
        height = -steplength
        for polygon in polygonlist:
            height = height+steplength
            for pointid in range(0, nvertperpolygon):
                point = polygon.exterior.coords[pointid]
                vertices.append(np.array([point[0], point[1], height]))
        for polygonid in range(0, npolygon-1):
            startingid = polygonid*nvertperpolygon
            startingidnext = (polygonid+1)*nvertperpolygon
            for vertid in range(0, nvertperpolygon):
                id0 = startingid+vertid
                id1 = startingid+vertid+1
                if vertid == nvertperpolygon-1:
                    id1 = startingid
                idnext0 = startingidnext+vertid
                idnext1 = startingidnext+vertid+1
                if vertid == nvertperpolygon-1:
                    idnext1 = startingidnext
                triangles.append(np.array([id0, id1, idnext0]))
                rawnormal0 = np.cross(vertices[id1]-vertices[id0], vertices[idnext0]-vertices[id1])
                triangles.append(np.array([id1, idnext1, idnext0]))
                rawnormal1 = np.cross(vertices[idnext1]-vertices[id1], vertices[idnext0]-vertices[idnext1])
                rawnormal = rawnormal0+rawnormal1
                facenormals.append(rawnormal/np.linalg.norm(rawnormal))
                facenormals.append(rawnormal/np.linalg.norm(rawnormal))

        # top and bottom
        for vertid in range(2, nvertperpolygon):
            triangles.append(np.array([0, vertid, vertid-1]))
            rawnormal = np.cross(vertices[vertid]-vertices[0], vertices[vertid-1]-vertices[vertid])
            facenormals.append(rawnormal/np.linalg.norm(rawnormal))
        nverts = len(vertices)
        for vertid in range(2, nvertperpolygon):
            triangles.append(np.array([nverts-1, nverts-1-vertid, nverts-vertid]))
            rawnormal = np.cross(vertices[nverts-1-vertid]-vertices[nverts-1],
                                 vertices[nverts-vertid]-vertices[nverts-1-vertid])
            facenormals.append(rawnormal/np.linalg.norm(rawnormal))

        cobnp = pg.packpandanp(np.asarray(vertices), np.asarray(facenormals), np.asarray(triangles))
        return cobnp

    def genCobtDisnp(self, polygonlist, steplength):
        """
        generate a mesh model for polygonlistSeg, each polygonis a separate one

        :param polygonlist:
        :param color:
        :return:

        author: weiwei
        date: 20170924, Vancouver
        """

        npolygon = len(polygonlist)
        vertices = []
        facenormals = []
        triangles = []
        nvertperpolygon = len(polygonlist[0].exterior.coords)-1
        height = -steplength
        for polygon in polygonlist:
            height = height+steplength
            for pointid in range(0, nvertperpolygon):
                point = polygon.exterior.coords[pointid]
                vertices.append(np.array([point[0], point[1], height]))
        for polygonid in range(0, npolygon):
            startingid = polygonid*nvertperpolygon
            for vertid in range(1, nvertperpolygon-1):
                id0 = startingid
                id1 = startingid+vertid
                id2 = startingid+vertid+1
                if vertid == nvertperpolygon-1:
                    id1 = startingid
                triangles.append(np.array([id0, id2, id1]))
                rawnormal = -np.cross(vertices[id1]-vertices[id0], vertices[id2]-vertices[id1])
                facenormals.append(rawnormal/np.linalg.norm(rawnormal))
        polygonnps = []
        height = -steplength
        for polygon in polygonlist:
            height = height+steplength
            polygonnps.append(self.genPolygonsnp(polygon, height, color=[0,0,0,1], thickness=2)[0])

        cobnp = pg.packpandanp(np.asarray(vertices), np.asarray(facenormals[::-1]), np.asarray(triangles[::-1]))
        return [cobnp, polygonnps]

    def genLinesegsnp(self, verts3d, color = [], thickness = 20.0):
        return pg.genLinesegsnp(verts3d, color, thickness)

    def plotLinesegsnp(self, verts3d, color = [], thickness = 20.0):
        return pg.genLinesegsnp(verts3d, color, thickness)

    def genPointsnp(self, point, height, color = [], size = 2.0):
        verts3d = [[point.x, point.y, height]]
        return pg.genPntsnp(verts3d, color, size)

    def genOneDCurve(self, polygon, fingerpoint0, fingerpoint1, constraindirect = [0,1]):
        """
        compute one dimensional curve given the initial coordinates of polygon, fingerpoint0, and fingerpoint1

        :param polygon: follows shapely definitioin
        :param fingerpoint0: point, follows shapely def
        :param fingerpoint1: same
        :param constraindirect: the direction of constraint (default y+)
        :return: [onedcurves, polygonxoncurves, fingerp0s3d, fingerp1s3d], each element is a list (multi-sections)

        author: weiwei
        date: 20170509
        """

        # obstacles
        cobtfgr0 = self.genMinkovPnt(polygon, fingerpoint0)
        cobtfgr1 = self.genMinkovPnt(polygon, fingerpoint1)

        # onedcurves saves multiple onedcurve
        onedcurves = []
        onedcurve = []
        lastxy = []
        fingerp0s3d = []
        fingerp03d = []
        fingerp1s3d = []
        fingerp13d = []
        polygonxoncurves = []
        polygonxoncurve = []
        for height in range(0, self.heightrange, self.steplength):
            rotedcobsp0 = self.rotate(cobtfgr0, height / self.scale)
            rotedcobsp1 = self.rotate(cobtfgr1, height / self.scale)
            cobsintersections = rotedcobsp0.boundary.intersection(rotedcobsp1.boundary)
            if not cobsintersections.is_empty:
                if len(lastxy) != 0:
                    mindist = 9999999999999.9
                    minlastxy = []
                    for cobsintersection in cobsintersections:
                        if cobsintersection.geom_type == "Point":
                            diffx = cobsintersection.x - lastxy[0]
                            diffy = cobsintersection.y - lastxy[1]
                            diff = math.sqrt(diffx * diffx + diffy * diffy)
                            if diff < mindist:
                                mindist = diff
                                minlastxy = [cobsintersection.x, cobsintersection.y]
                        elif cobsintersection.geom_type == "LineString":
                            for vert in cobsintersection.coords:
                                diffx = vert[0] - lastxy[0]
                                diffy = vert[1] - lastxy[1]
                                diff = math.sqrt(diffx * diffx + diffy * diffy)
                                if diff < mindist:
                                    mindist = diff
                                    minlastxy = [vert[0], vert[1]]
                    lastxy = minlastxy
                else:
                    lastxy = [-constraindirect[0]*1000000.0, -constraindirect[1]*1000000.0]
                    for cobsintersection in cobsintersections:
                        # print cobsintersection.geom_type
                        if cobsintersection.geom_type == "Point":
                            vert = [cobsintersection.x, cobsintersection.y]
                            projinter = vert[0]*constraindirect[0]+vert[1]*constraindirect[1]
                            projlastxy = lastxy[0]*constraindirect[0] + lastxy[1]*constraindirect[1]
                            if projinter >= projlastxy:
                                lastxy = vert
                        elif cobsintersection.geom_type == "LineString":
                            for vert in cobsintersection.coords:
                                projinter = vert[0]*constraindirect[0]+vert[1]*constraindirect[1]
                                projlastxy = lastxy[0]*constraindirect[0] + lastxy[1]*constraindirect[1]
                                if projinter >= projlastxy:
                                    lastxy = vert
                onedcurve.append([lastxy[0], lastxy[1], height])
                fingerp03d.append([fingerpoint0.x, fingerpoint0.y, height])
                fingerp13d.append([fingerpoint1.x, fingerpoint1.y, height])

                polygonx = self.rotate(polygon, height / self.scale)
                polygonx = self.translate(polygonx, Point([lastxy[0], lastxy[1]]))
                polygonxoncurve.append(polygonx)

                # validate the case with only one intersection point
                lastxy = []

                # if reach the end of rotation, save onedcurve into onedcurves and start a new onedcurve
                if height >= self.heightrange - self.steplength:
                    onedcurves.append(onedcurve)
                    onedcurve = []
                    fingerp0s3d.append(fingerp03d)
                    fingerp03d = []
                    fingerp1s3d.append(fingerp13d)
                    fingerp13d = []
                    polygonxoncurves.append(polygonxoncurve)
                    polygonxoncurve = []
                    lastxy = []
            else:
                # if intersection disappeared, save onedcurve into onedcurves and start a new onedcurve
                onedcurves.append(onedcurve)
                onedcurve = []
                polygonxoncurves.append(polygonxoncurve)
                polygonxoncurve = []
                lastxy = []

        return [onedcurves, polygonxoncurves, fingerp0s3d, fingerp1s3d]

    def genOneDCurveSeg(self, polygon, segfgr, sppoint, constraindirect = [0,1]):
        """
        compute one dimensional curve given the initial coordinates of polygon, fingerpoint0, and fingerpoint1

        :param polygon: follows shapely definitioin
        :param segfgr: a segment in the form of [Point, Point]
        :param sppoint: a support Point
        :param constraindirect: the direction of constraint (default y+)
        :return: [onedcurves, polygonxoncurves, segfgr3d, sppoint3d], each element is a list (multi-sections)

        author: weiwei
        date: 20170525
        """

        # obstacles
        cobtspbase = self.genMinkovPnt(polygon, sppoint)

        # onedcurves saves multiple onedcurve
        lastxy = []
        # upper: on all curves, lower: on one curve
        onedcurves = []
        onedcurve = []
        segfgrs3d = []
        segfgr3d = []
        sppoints3d = []
        sppoint3d = []
        polygonxoncurves = []
        polygonxoncurve = []
        for height in range(0, self.heightrange, self.steplength):
            cobtseg = self.genMinkovSeg(self.rotate(polygon, height/self.scale), segfgr)
            cobtsp = self.rotate(cobtspbase, height / self.scale)
            cobsintersections = cobtseg.boundary.intersection(cobtsp.boundary)
            if not cobsintersections.is_empty:
                if len(lastxy) != 0:
                    mindist = 9999999999999.9
                    minlastxy = []
                    for cobsintersection in cobsintersections:
                        if cobsintersection.geom_type == "Point":
                            diffx = cobsintersection.x - lastxy[0]
                            diffy = cobsintersection.y - lastxy[1]
                            diff = math.sqrt(diffx * diffx + diffy * diffy)
                            if diff < mindist:
                                mindist = diff
                                minlastxy = [cobsintersection.x, cobsintersection.y]
                        elif cobsintersection.geom_type == "LineString":
                            for vert in cobsintersection.coords:
                                diffx = vert[0] - lastxy[0]
                                diffy = vert[1] - lastxy[1]
                                diff = math.sqrt(diffx * diffx + diffy * diffy)
                                if diff < mindist:
                                    mindist = diff
                                    minlastxy = [vert[0], vert[1]]
                    lastxy = minlastxy
                else:
                    lastxy = [-constraindirect[0]*10000000.0, -constraindirect[1]*10000000.0]
                    for cobsintersection in cobsintersections:
                        projinter = cobsintersection.x*constraindirect[0]+cobsintersection.y*constraindirect[1]
                        projlastxy = lastxy[0]*constraindirect[0] + lastxy[1]*constraindirect[1]
                        if projinter >= projlastxy:
                            lastxy = [cobsintersection.x, cobsintersection.y]
                onedcurve.append([lastxy[0], lastxy[1], height])
                segfgr3d.append([[segfgr[0].x, segfgr[0].y, height], [segfgr[1].x, segfgr[1].y, height]])
                sppoint3d.append([sppoint.x, sppoint.y, height])

                polygonx = self.rotate(polygon, height / self.scale)
                polygonx = self.translate(polygonx, Point([lastxy[0], lastxy[1]]))
                polygonxoncurve.append(polygonx)

                # validate the case with only one intersection point
                lastxy = []

                # if reach the end of rotation, save onedcurve into onedcurves and start a new onedcurve
                if height > heightrange - steplength:
                    onedcurves.append(onedcurve)
                    onedcurve = []
                    segfgrs3d.append(segfgr3d)
                    segfgr3d = []
                    sppoints3d.append(sppoint3d)
                    sppoint3d = []
                    polygonxoncurves.append(polygonxoncurve)
                    polygonxoncurve = []
                    lastxy = []
            else:
                # if intersection disappeared, save onedcurve into onedcurves and start a new onedcurve
                onedcurves.append(onedcurve)
                onedcurve = []
                polygonxoncurves.append(polygonxoncurve)
                polygonxoncurve = []
                segfgrs3d.append(segfgr3d)
                segfgr3d = []
                sppoints3d.append(sppoint3d)
                sppoint3d = []
                lastxy = []
        return [onedcurves, polygonxoncurves, segfgrs3d, sppoints3d]

    def cavities(self, onedcurve):
        """
        find the cavities of onedcurve
        cavities are represented by triples (leftlocalmax, localmin, rightlocalmax)

        :param onedcurve:
        :return: [cavities0, cavities1, ...]

        author: weiwei
        date: 20170511
        """

        onedcavities = []
        cavity = []
        inc = False
        dec = False
        for i, point in enumerate(onedcurve):
            if i >= 1:
                prepoint = onedcurve[i-1]
                if point[1] > prepoint[1] and dec:
                    # local minima
                    inc = True
                    dec = False
                    if len(cavity) == 1:
                        cavity.append(i-1)
                elif point[1] > prepoint[1] and not inc:
                    inc = True
                    dec = False
                if point[1] < prepoint[1] and inc:
                    # local maxima
                    inc = False
                    dec = True
                    if len(cavity) == 0:
                        cavity.append(i-1)
                    elif len(cavity) == 2:
                        cavity.append(i-1)
                        onedcavities.append(cavity)
                        cavity = [i-1]
                        inc = False
                        dec = True
                elif point[1] < prepoint[1] and not dec:
                    inc = False
                    dec = True
        return onedcavities

    def genCobt(self, polygon, point):
        cobtslice = self.genMinkovPnt(polygon, point)
        cobtsdict = {}
        for height in range(0,self.heightrange,self.steplength):
            rotedcobtslice = self.rotate(cobtslice, height/self.scale)
            cobtsdict[height] = rotedcobtslice
        return cobtsdict

    def genCobtSeg(self, polygon, segment):
        cobtsdict = {}
        for height in range(0,self.heightrange,self.steplength):
            rotedcobtslice = self.genMinkovSeg(self.rotate(polygon, height/self.scale), segment)
            cobtsdict[height] = rotedcobtslice
        return cobtsdict

if __name__=="__main__":
    base = pc.World(camp=[0,0,5800], lookatp=[0,0,1800])

    polygon = Polygon([[0.0,0.0],
                       [1000.0,0.0],
                       [2000.0,1000.0],
                       [1000.0,1500.0],
                       [0.0,1000.0]])

    sppoint1 = Point(900.0,0.0)
    sppoint0 = Point(1200.0,200.0)
    sppoint2 = Point(0.0,100.0)

    cobtsp0 = genMinkovPnt(polygon, sppoint0)
    cobtsp1 = genMinkovPnt(polygon, sppoint1)
    cobtsp2 = genMinkovPnt(polygon, sppoint2)

    polygonsnp = genPolygonsnp(polygon, 0.0, color = [0,1,1,1], thickness=3)
    for polygonnp in polygonsnp:
        polygonnp.reparentTo(base.render)

    # obstacles
    stratcurveverts3d = []
    lastxy = []
    sppoint13d = []
    sppoint23d = []
    heightrange = 3600
    steplength = 31
    # colors
    objcolorarray = pg.randomColorArray(int(heightrange/float(steplength))+1)
    for height in range(0,heightrange,steplength):
        if height/steplength > 3:
            pg.plotLinesegs(base.render, stratcurveverts3d, rgba=[1.0, 1.0, 0.0, 1], thickness=30)
            stratcurveverts3d = []
            lastxy = []
            break
        rotedcobsp0 = rotate(cobtsp0, height/10.0, sppoint0)
        rotedcobsp1 = rotate(cobtsp1, height/10.0, sppoint1)
        rotedcobsp2 = rotate(cobtsp2, height/10.0, sppoint2)
        # rotedcobtspunion = rotedcobsp0.union(rotedcobsp1).union(rotedcobsp2)
        rotedcobtspunion = rotedcobsp1.union(rotedcobsp2)
        rotedcobspolygonsnp = genPolygonsnp(rotedcobtspunion, height, color = [1.0,0,0,.3], thickness=1)
        # for rotedcobspolygonnp in rotedcobspolygonsnp:
            # rotedcobspolygonnp.reparentTo(base.render)
        # stratum curve
        cobsintersections = rotedcobsp1.boundary.intersection(rotedcobsp2.boundary)
        if not cobsintersections.is_empty:
            if len(lastxy):
                mindist = 9999999999999.9
                minlastxy = []
                for cobsintersection in cobsintersections:
                    if cobsintersection.geom_type == "Point":
                        diffx = cobsintersection.x-lastxy[0]
                        diffy = cobsintersection.y-lastxy[1]
                        diff = math.sqrt(diffx*diffx+diffy*diffy)
                        if diff < mindist:
                            mindist = diff
                            minlastxy = [cobsintersection.x, cobsintersection.y]
                    elif cobsintersection.geom_type == "LineString":
                        for vert in cobsintersection.coords:
                            diffx = vert[0] - lastxy[0]
                            diffy = vert[1] - lastxy[1]
                            diff = math.sqrt(diffx*diffx+diffy*diffy)
                            if diff < mindist:
                                mindist = diff
                                minlastxy = [vert[0], vert[1]]
                lastxy = minlastxy
            else:
                lastxy = [-9999999999999.9, -9999999999999.9]
                for cobsintersection in cobsintersections:
                    # print cobsintersection
                    if cobsintersection.y > lastxy[1]:
                        lastxy = [cobsintersection.x, cobsintersection.y]
            stratcurveverts3d.append([lastxy[0], lastxy[1], height])

            sppoint13d.append([sppoint1.x, sppoint1.y, height])
            sppoint23d.append([sppoint2.x, sppoint2.y, height])
            polygonx = rotate(polygon, height/10.0)
            polygonx = translate(polygonx, Point([lastxy[0], lastxy[1]]))
            # pg.plotDumbbell(base.render, spos = [lastxy[0], lastxy[1], height], epos = [lastxy[0], lastxy[1], height], length = 0, thickness=30 )
            polygonsnp = genPolygonsnp(polygonx, height, color=objcolorarray[int(height/float(steplength))], thickness=20)
            for polygonnp in polygonsnp:
                polygonnp.reparentTo(base.render)
            if height > heightrange-steplength:
                pg.plotLinesegs(base.render, stratcurveverts3d, rgba = [1.0,1.0,0.0,1], thickness=30)
                stratcurveverts3d = []
                lastxy = []
        else:
            pg.plotLinesegs(base.render, stratcurveverts3d, rgba = [1.0,1.0,0.0,1], thickness=30)
            stratcurveverts3d = []
            lastxy = []

    pg.plotLinesegs(base.render, sppoint13d, rgba=[.5,.2,.5,.3], thickness=30, headscale = 3)
    pg.plotLinesegs(base.render, sppoint23d, rgba=[.2,.5,.5,.3], thickness=30, headscale = 3)

    pg.plotAxisSelf(base.render, length=1500, thickness = 70)
    base.run()