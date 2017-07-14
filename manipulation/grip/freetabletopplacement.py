#!/usr/bin/python

import os

import MySQLdb as mdb
import numpy as np
from manipulation.grip.robotiq85 import rtq85nm
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletDebugNode
from panda3d.core import *
from shapely.geometry import LinearRing
from shapely.geometry import Point
from shapely.geometry import Polygon

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pandageom
import trimesh
from pandaplotutils import pandageom as pg
from utils import collisiondetection as cd
from utils import dbcvt as dc
from utils import robotmath as rm
from database import dbaccess as db

class FreeTabletopPlacement(object):
    """
    manipulation.freetabletopplacement doesn't take into account
    the position and orientation of the object
    it is "free" in position and rotation around z axis
    in contrast, each item in regrasp.tabletopplacements
    has different position and orientation
    it is at a specific pose in the workspace
    To clearly indicate the difference, "free" is attached
    to the front of "freetabletopplacement"
    "s" is attached to the end of "tabletopplacements"
    """

    def __init__(self, objpath, handpkg, gdb):
        self.objtrimesh=trimesh.load_mesh(objpath)
        self.objcom = self.objtrimesh.center_mass
        self.objtrimeshconv=self.objtrimesh.convex_hull
        # oc means object convex
        self.ocfacets, self.ocfacetnormals = self.objtrimeshconv.facets_over(.9999)

        # for dbaccess
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

        # use two bulletworld, one for the ray, the other for the tabletop
        self.bulletworldray = BulletWorld()
        self.bulletworldhp = BulletWorld()
        # plane to remove hand
        self.planebullnode = cd.genCollisionPlane(offset=0)
        self.bulletworldhp.attachRigidBody(self.planebullnode)

        self.handpkg = handpkg
        self.handname = handpkg.getHandName()
        self.hand = handpkg.newHandNM(hndcolor=[0,1,0,.1])
        # self.rtq85hnd = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, .1])

        # for dbsave
        # each tpsmat4 corresponds to a set of tpsgripcontacts/tpsgripnormals/tpsgripjawwidth list
        self.tpsmat4s = None
        self.tpsgripcontacts = None
        self.tpsgripnormals = None
        self.tpsgripjawwidth = None

        # for ocFacetShow
        self.counter = 0

        self.gdb = gdb
        self.loadFreeAirGrip()

    def loadFreeAirGrip(self):
        """
        load self.freegripid, etc. from mysqldatabase

        :param gdb: an object of the database.GraspDB class
        :return:

        author: weiwei
        date: 20170110
        """

        freeairgripdata = self.gdb.loadFreeAirGrip(self.dbobjname, handname = self.handname)
        if freeairgripdata is None:
            raise ValueError("Plan the freeairgrip first!")

        self.freegripid = freeairgripdata[0]
        self.freegripcontacts = freeairgripdata[1]
        self.freegripnormals = freeairgripdata[2]
        self.freegriprotmats = freeairgripdata[3]
        self.freegripjawwidth = freeairgripdata[4]

    def loadFreeTabletopPlacement(self):
        """
        load free tabletopplacements

        :return:
        """
        tpsmat4s = self.gdb.loadFreeTabletopPlacement(self.dbobjname)
        if tpsmat4s is not None:
            self.tpsmat4s = tpsmat4s
            return True
        else:
            self.tpsmat4s = []
            return False

    def removebadfacets(self, base, doverh=.1):
        """
        remove the facets that cannot support stable placements

        :param: doverh: d is the distance of mproj to supportfacet boundary, h is the height of com
                when fh>dmg, the object tends to fall over. setting doverh to 0.033 means
                when f>0.1mg, the object is judged to be unstable
        :return:

        author: weiwei
        date: 20161213
        """
        self.tpsmat4s = []

        for i in range(len(self.ocfacets)):
            geom = pg.packpandageom(self.objtrimeshconv.vertices,
                                           self.objtrimeshconv.face_normals[self.ocfacets[i]],
                                           self.objtrimeshconv.faces[self.ocfacets[i]])
            geombullnode = cd.genCollisionMeshGeom(geom)
            self.bulletworldray.attachRigidBody(geombullnode)
            pFrom = Point3(self.objcom[0], self.objcom[1], self.objcom[2])
            pTo = self.objcom+self.ocfacetnormals[i]*99999
            pTo = Point3(pTo[0], pTo[1], pTo[2])
            result = self.bulletworldray.rayTestClosest(pFrom, pTo)
            self.bulletworldray.removeRigidBody(geombullnode)
            if result.hasHit():
                hitpos = result.getHitPos()

                facetinterpnt = np.array([hitpos[0],hitpos[1],hitpos[2]])
                facetnormal = np.array(self.ocfacetnormals[i])
                bdverts3d, bdverts2d, facetmat4 = pg.facetboundary(self.objtrimeshconv, self.ocfacets[i],
                                                                     facetinterpnt, facetnormal)
                facetp = Polygon(bdverts2d)
                facetinterpnt2d = rm.transformmat4(facetmat4, facetinterpnt)[:2]
                apntpnt = Point(facetinterpnt2d[0], facetinterpnt2d[1])
                dist2p = apntpnt.distance(facetp.exterior)
                dist2c = np.linalg.norm(np.array([hitpos[0],hitpos[1],hitpos[2]])-np.array([pFrom[0],pFrom[1],pFrom[2]]))
                if dist2p/dist2c > doverh:
                    # hit and stable
                    self.tpsmat4s.append(pg.cvtMat4np4(facetmat4))

    def gentpsgrip(self, base):
        """
        Originally the code of this function is embedded in the removebadfacet function
        It is separated on 20170608 to enable common usage of placements for different hands

        :return:

        author: weiwei
        date: 20170608
        """

        self.tpsgripcontacts = []
        self.tpsgripnormals = []
        self.tpsgriprotmats = []
        self.tpsgripjawwidth = []
        # the id of the grip in freeair
        self.tpsgripidfreeair = []

        for i in range(len(self.tpsmat4s)):
            self.tpsgripcontacts.append([])
            self.tpsgripnormals.append([])
            self.tpsgriprotmats.append([])
            self.tpsgripjawwidth.append([])
            self.tpsgripidfreeair.append([])
            for j, rotmat in enumerate(self.freegriprotmats):
                tpsgriprotmat = rotmat * self.tpsmat4s[i]
                # check if the hand collide with tabletop
                # tmprtq85 = self.rtq85hnd
                tmphnd = self.hand
                # tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, 1])
                initmat = tmphnd.getMat()
                initjawwidth = tmphnd.jawwidth
                # open the hand to ensure it doesnt collide with surrounding obstacles
                # tmprtq85.setJawwidth(self.freegripjawwidth[j])
                tmphnd.setJawwidth(80)
                tmphnd.setMat(tpsgriprotmat)
                # add hand model to bulletworld
                hndbullnode = cd.genCollisionMeshMultiNp(tmphnd.handnp)
                result = self.bulletworldhp.contactTest(hndbullnode)
                # print result.getNumContacts()
                if not result.getNumContacts():
                    self.tpsgriprotmats[-1].append(tpsgriprotmat)
                    cct0 = self.tpsmat4s[i].xformPoint(self.freegripcontacts[j][0])
                    cct1 = self.tpsmat4s[i].xformPoint(self.freegripcontacts[j][1])
                    self.tpsgripcontacts[-1].append([cct0, cct1])
                    cctn0 = self.tpsmat4s[i].xformVec(self.freegripnormals[j][0])
                    cctn1 = self.tpsmat4s[i].xformVec(self.freegripnormals[j][1])
                    self.tpsgripnormals[-1].append([cctn0, cctn1])
                    self.tpsgripjawwidth[-1].append(self.freegripjawwidth[j])
                    self.tpsgripidfreeair[-1].append(self.freegripid[j])
                tmphnd.setMat(initmat)
                tmphnd.setJawwidth(initjawwidth)

    def saveToDB(self):
        """
        save freetabletopplacement

        manipulation.freetabletopplacement doesn't take into account the position and orientation of the object
        it is "free" in position and rotation around z axis
        in contrast, each item in regrasp.tabletopplacements has different position and orientation
        it is at a specific pose in the workspace
        To clearly indicate the difference, "free" is attached to the front of "freetabletopplacement"
        "s" is attached to the end of "tabletopplacements"

        :param discretesize:
        :param gdb:
        :return:

        author: weiwei
        date: 20170111
        """

        # save freetabletopplacement
        sql = "SELECT * FROM freetabletopplacement,object WHERE freetabletopplacement.idobject = object.idobject \
                AND object.name LIKE '%s'" % self.dbobjname
        result = self.gdb.execute(sql)
        if len(result) == 0:
            # the fretabletopplacements for the self.dbobjname is not saved
            sql = "INSERT INTO freetabletopplacement(rotmat, idobject) VALUES "
            for i in range(len(self.tpsmat4s)):
                sql += "('%s', (SELECT idobject FROM object WHERE name LIKE '%s')), " % \
                       (dc.mat4ToStr(self.tpsmat4s[i]), self.dbobjname)
            sql = sql[:-2] + ";"
            self.gdb.execute(sql)
        else:
            print "Freetabletopplacement already exist!"

        # save freetabletopgrip
        idhand = gdb.loadIdHand(self.handname)
        sql = "SELECT * FROM freetabletopgrip,freetabletopplacement,freeairgrip,object WHERE \
                freetabletopgrip.idfreetabletopplacement = freetabletopplacement.idfreetabletopplacement AND \
                freetabletopgrip.idfreeairgrip = freeairgrip.idfreeairgrip AND \
                freetabletopplacement.idobject = object.idobject AND \
                object.name LIKE '%s' AND freeairgrip.idhand = %d" % (self.dbobjname, idhand)
        result = self.gdb.execute(sql)
        if len(result) == 0:
            for i in range(len(self.tpsmat4s)):
                sql = "SELECT freetabletopplacement.idfreetabletopplacement FROM freetabletopplacement,object WHERE \
                        freetabletopplacement.rotmat LIKE '%s' AND \
                        object.name LIKE '%s'" % (dc.mat4ToStr(self.tpsmat4s[i]), self.dbobjname)
                result = self.gdb.execute(sql)[0]
                print result
                if len(result) != 0:
                    idfreetabletopplacement = result[0]
                    # note self.tpsgriprotmats[i] might be empty (no cd-free grasps)
                    if len(self.tpsgriprotmats[i]) != 0:
                        sql = "INSERT INTO freetabletopgrip(contactpoint0, contactpoint1, contactnormal0, contactnormal1, \
                                rotmat, jawwidth, idfreetabletopplacement, idfreeairgrip) VALUES "
                        for j in range(len(self.tpsgriprotmats[i])):
                            cct0 = self.tpsgripcontacts[i][j][0]
                            cct1 = self.tpsgripcontacts[i][j][1]
                            cctn0 = self.tpsgripnormals[i][j][0]
                            cctn1 = self.tpsgripnormals[i][j][1]
                            sql += "('%s', '%s', '%s', '%s', '%s', '%s', %d, %d), " % \
                                   (dc.v3ToStr(cct0), dc.v3ToStr(cct1), dc.v3ToStr(cctn0), dc.v3ToStr(cctn1), \
                                    dc.mat4ToStr(self.tpsgriprotmats[i][j]), str(self.tpsgripjawwidth[i][j]), \
                                    idfreetabletopplacement, self.tpsgripidfreeair[i][j])
                        sql = sql[:-2] + ";"
                        self.gdb.execute(sql)
        else:
            print "Freetabletopgrip already exist!"


    def removebadfacetsshow(self, base, doverh=.1):
        """
        remove the facets that cannot support stable placements

        :param: doverh: d is the distance of mproj to supportfacet boundary, h is the height of com
                when fh>dmg, the object tends to fall over. setting doverh to 0.033 means
                when f>0.1mg, the object is judged to be unstable
        :return:

        author: weiwei
        date: 20161213
        """

        plotoffsetfp = 10
        # print self.counter

        if self.counter < len(self.ocfacets):
            i = self.counter
        # for i in range(len(self.ocfacets)):
            geom = pg.packpandageom(self.objtrimeshconv.vertices,
                                           self.objtrimeshconv.face_normals[self.ocfacets[i]],
                                           self.objtrimeshconv.faces[self.ocfacets[i]])
            geombullnode = cd.genCollisionMeshGeom(geom)
            self.bulletworldray.attachRigidBody(geombullnode)
            pFrom = Point3(self.objcom[0], self.objcom[1], self.objcom[2])
            pTo = self.objcom+self.ocfacetnormals[i]*99999
            pTo = Point3(pTo[0], pTo[1], pTo[2])
            result = self.bulletworldray.rayTestClosest(pFrom, pTo)
            self.bulletworldray.removeRigidBody(geombullnode)
            if result.hasHit():
                hitpos = result.getHitPos()
                pg.plotArrow(base.render, spos=self.objcom,
                             epos = self.objcom+self.ocfacetnormals[i], length=100)

                facetinterpnt = np.array([hitpos[0],hitpos[1],hitpos[2]])
                facetnormal = np.array(self.ocfacetnormals[i])
                bdverts3d, bdverts2d, facetmat4 = pg.facetboundary(self.objtrimeshconv, self.ocfacets[i],
                                                                     facetinterpnt, facetnormal)
                for j in range(len(bdverts3d)-1):
                    spos = bdverts3d[j]
                    epos = bdverts3d[j+1]
                    pg.plotStick(base.render, spos, epos, thickness = 1, rgba=[.5,.5,.5,1])

                facetp = Polygon(bdverts2d)
                facetinterpnt2d = rm.transformmat4(facetmat4, facetinterpnt)[:2]
                apntpnt = Point(facetinterpnt2d[0], facetinterpnt2d[1])
                dist2p = apntpnt.distance(facetp.exterior)
                dist2c = np.linalg.norm(np.array([hitpos[0],hitpos[1],hitpos[2]])-np.array([pFrom[0],pFrom[1],pFrom[2]]))
                print dist2p
                print dist2c
                print dist2p/dist2c
                if dist2p/dist2c < doverh:
                    print "not stable"
                    # return
                else:
                    pol_ext = LinearRing(bdverts2d)
                    d = pol_ext.project(apntpnt)
                    p = pol_ext.interpolate(d)
                    closest_point_coords = list(p.coords)[0]
                    closep = np.array([closest_point_coords[0], closest_point_coords[1], 0])
                    closep3d = rm.transformmat4(rm.homoinverse(facetmat4), closep)[:3]
                    pg.plotDumbbell(base.render, spos=facetinterpnt, epos=closep3d, thickness=1.5, rgba=[0,0,1,1])

                    for j in range(len(bdverts3d)-1):
                        spos = bdverts3d[j]
                        epos = bdverts3d[j+1]
                        pg.plotStick(base.render, spos, epos, thickness = 1.5, rgba=[0,1,0,1])

                    # geomoff = pg.packpandageom(self.objtrimeshconv.vertices +
                    #                                np.tile(plotoffsetfp * self.ocfacetnormals[i],
                    #                                        [self.objtrimeshconv.vertices.shape[0], 1]),
                    #                         self.objtrimeshconv.face_normals[self.ocfacets[i]],
                    #                         self.objtrimeshconv.faces[self.ocfacets[i]])
                    #
                    # nodeoff = GeomNode('supportfacet')
                    # nodeoff.addGeom(geomoff)
                    # staroff = NodePath('supportfacet')
                    # staroff.attachNewNode(nodeoff)
                    # staroff.setColor(Vec4(1,0,1,1))
                    # staroff.setTransparency(TransparencyAttrib.MAlpha)
                    # staroff.setTwoSided(True)
                    # staroff.reparentTo(base.render)
            self.counter+=1
        else:
            self.counter=0

    def grpshow(self, base):

        sql = "SELECT freetabletopplacement.idfreetabletopplacement, freetabletopplacement.rotmat \
                       FROM freetabletopplacement,object WHERE \
                       freetabletopplacement.idobject = object.idobject AND object.name LIKE '%s'" % self.dbobjname
        result = self.gdb.execute(sql)
        if len(result) != 0:
            idfreetabletopplacement = int(result[3][0])
            objrotmat  = dc.strToMat4(result[3][1])
            # show object
            geom = pg.packpandageom(self.objtrimesh.vertices,
                                    self.objtrimesh.face_normals,
                                    self.objtrimesh.faces)
            node = GeomNode('obj')
            node.addGeom(geom)
            star = NodePath('obj')
            star.attachNewNode(node)
            star.setColor(Vec4(.77,0.67,0,1))
            star.setTransparency(TransparencyAttrib.MAlpha)
            star.setMat(objrotmat)
            star.reparentTo(base.render)
            sql = "SELECT freetabletopgrip.rotmat, freetabletopgrip.jawwidth FROM freetabletopgrip WHERE \
                                freetabletopgrip.idfreetabletopplacement=%d" % idfreetabletopplacement
            result = self.gdb.execute(sql)
            for resultrow in result:
                hndrotmat = dc.strToMat4(resultrow[0])
                hndjawwidth = float(resultrow[1])
                # show grasps
                tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[0, 1, 0, .1])
                tmprtq85.setMat(hndrotmat)
                tmprtq85.setJawwidth(hndjawwidth)
                # tmprtq85.setJawwidth(80)
                tmprtq85.reparentTo(base.render)

    def showOnePlacementAndAssociatedGrips(self, base):
        """
        show one placement and its associated grasps
        :param base:
        :return:
        """

        for i in range(len(self.tpsmat4s)):
            if i == 1:
                objrotmat  = self.tpsmat4s[i]
                # objrotmat.setRow(0, -objrotmat.getRow3(0))
                rotzmat = Mat4.rotateMat(0, Vec3(0,0,1))
                objrotmat = objrotmat*rotzmat
                # show object
                geom = pg.packpandageom(self.objtrimesh.vertices,
                                        self.objtrimesh.face_normals,
                                        self.objtrimesh.faces)
                node = GeomNode('obj')
                node.addGeom(geom)
                star = NodePath('obj')
                star.attachNewNode(node)
                star.setColor(Vec4(.7,0.3,0,1))
                star.setTransparency(TransparencyAttrib.MAlpha)
                star.setMat(objrotmat)
                star.reparentTo(base.render)
                # for j in range(len(self.tpsgriprotmats[i])):
                for j in range(13,14):
                    hndrotmat = self.tpsgriprotmats[i][j]
                    hndjawwidth = self.tpsgripjawwidth[i][j]
                    # show grasps
                    tmphnd = self.handpkg.newHandNM(hndcolor=[0, 1, 0, .5])
                    tmphnd.setMat(hndrotmat)
                    tmphnd.setJawwidth(hndjawwidth)
                    # tmprtq85.setJawwidth(80)
                    tmphnd.reparentTo(base.render)

    def ocfacetshow(self, base):
        print self.objcom

        npf = base.render.find("**/supportfacet")
        if npf:
            npf.removeNode()

        plotoffsetfp = 10

        print self.counter
        print len(self.ocfacets)
        if self.counter < len(self.ocfacets):
            geom = pandageom.packpandageom(self.objtrimeshconv.vertices+
                                           np.tile(plotoffsetfp*self.ocfacetnormals[self.counter],
                                                   [self.objtrimeshconv.vertices.shape[0],1]),
                                           self.objtrimeshconv.face_normals[self.ocfacets[self.counter]],
                                           self.objtrimeshconv.faces[self.ocfacets[self.counter]])
            # geom = pandageom.packpandageom(self.objtrimeshconv.vertices,
            #                                self.objtrimeshconv.face_normals,
            #                                self.objtrimeshconv.faces)
            node = GeomNode('supportfacet')
            node.addGeom(geom)
            star = NodePath('supportfacet')
            star.attachNewNode(node)
            star.setColor(Vec4(1,0,1,1))
            star.setTransparency(TransparencyAttrib.MAlpha)
            star.setTwoSided(True)
            star.reparentTo(base.render)
            self.counter+=1
        else:
            self.counter = 0

if __name__ == '__main__':

    base = pandactrl.World(camp=[700,300,700], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(this_dir, "objects", "ttube.stl")
    # objpath = os.path.join(this_dir, "objects", "tool.stl")
    # objpath = os.path.join(this_dir, "objects", "tool2.stl")
    # objpath = os.path.join(this_dir, "objects", "planewheel.stl")
    # objpath = os.path.join(this_dir, "objects", "planelowerbody.stl")
    # objpath = os.path.join(this_dir, "objects", "planefrontstay.stl")
    # objpath = os.path.join(this_dir, "objects", "planerearstay.stl")
    print objpath

    from manipulation.grip.hrp5three import hrp5threenm
    # handpkg = hrp5threenm
    handpkg = rtq85nm
    gdb = db.GraspDB()
    tps = FreeTabletopPlacement(objpath, handpkg, gdb)

    # objpath0 = os.path.join(this_dir, "objects", "ttube.stl")
    # objpath1 = os.path.join(this_dir, "objects", "tool.stl")
    # objpath2 = os.path.join(this_dir, "objects", "planewheel.stl")
    # objpath3 = os.path.join(this_dir, "objects", "planelowerbody.stl")
    # objpath4 = os.path.join(this_dir, "objects", "planefrontstay.stl")
    # objpath5 = os.path.join(this_dir, "objects", "planerearstay.stl")
    # objpaths = [objpath0, objpath1, objpath2, objpath3, objpath4, objpath5]
    # import time
    # fo = open("foo.txt", "w")
    # for objpath in objpaths:
    #     tic = time.clock()
    #     tps = FreeTabletopPlacement(objpath, gdb)
    #     tps.removebadfacets(base, doverh=.2)
    #     toc = time.clock()
    #     print toc-tic
    #     fo.write(os.path.basename(objpath)+' '+str(toc-tic)+'\n')
    # fo.close()


    # # plot obj and its convexhull
    # geom = pandageom.packpandageom(tps.objtrimesh.vertices,
    #                                tps.objtrimesh.face_normals,
    #                                tps.objtrimesh.faces)
    # node = GeomNode('obj')
    # node.addGeom(geom)
    # star = NodePath('obj')
    # star.attachNewNode(node)
    # star.setColor(Vec4(1,0,0,1))
    # star.setTransparency(TransparencyAttrib.MAlpha)
    # # star.reparentTo(base.render)
    # geom = pandageom.packpandageom(tps.objtrimeshconv.vertices,
    #                                tps.objtrimeshconv.face_normals,
    #                                tps.objtrimeshconv.faces)
    # node = GeomNode('objconv')
    # node.addGeom(geom)
    # star = NodePath('objconv')
    # star.attachNewNode(node)
    # star.setColor(Vec4(0, 1, 0, .3))
    # star.setTransparency(TransparencyAttrib.MAlpha)
    # star.reparentTo(base.render)
    # def updateshow(task):
    #     # tps.ocfacetshow(base)
    #     tps.removebadfacetsshow(base, doverh=.1)
    #     return task.again
    # taskMgr.doMethodLater(.1, updateshow, "tickTask")

    # def updateworld(world, task):
    #     world.doPhysics(globalClock.getDt())
    #     return task.cont
    #
    if tps.loadFreeTabletopPlacement():
        pass
    else:
        tps.removebadfacets(base, doverh=.2)
    tps.gentpsgrip(base)
    tps.saveToDB()
    #
    # bullcldrnp = base.render.attachNewNode("bulletcollider")
    # debugNode = BulletDebugNode('Debug')
    # debugNode.showWireframe(True)
    # debugNP = bullcldrnp.attachNewNode(debugNode)
    # debugNP.show()
    #
    # tps.bulletworldhp.setDebugNode(debugNP.node())
    #
    # taskMgr.add(updateworld, "updateworld", extraArgs=[tps.bulletworldhp], appendTask=True)

    # tps.grpshow(base)
    tps.showOnePlacementAndAssociatedGrips(base)
    base.run()