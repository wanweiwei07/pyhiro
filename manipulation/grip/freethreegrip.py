#!/usr/bin/python

import os

import MySQLdb as mdb
import numpy as np
from manipulation.grip.robotiq85 import rtq85nm
from manipulation.grip.hrp5three import hrp5threenm
from panda3d.bullet import BulletWorld
from panda3d.core import *

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pg
from manipulation.grip import freegripcontactpairs as fgcp
from utils import collisiondetection as cd
from utils import dbcvt as dc
from utils import robotmath as rm
from database import dbaccess as db
from panda3d.bullet import BulletDebugNode

class Freethreegrip(fgcp.FreegripContactpairs):

    def __init__(self, objpath, handpkg, readser=False, torqueresist = 50):
        """
        initialization

        :param objpath: path of the object
        :param ser: True use pre-computed template file for debug (in order to debug large models like tool.stl
        :param torqueresist: the maximum allowable distance to com (see FreegripContactpairs.planContactpairs)

        author: weiwei
        date: 20161201, osaka
        """

        super(self.__class__, self).__init__(objpath, readser)
        self.handpkg = handpkg
        self.hand = handpkg.newHandNM(hndcolor=[0,1,0,.1])
        if readser is False:
            self.removeBadSamples()
            self.clusterFacetSamplesRNN(reduceRadius=2)
            self.planContactpairs(torqueresist, self.hand.jawwidthopen)
            self.saveSerialized("tmpcp.pickle")
        else:
            self.loadSerialized("tmpcp.pickle", objpath)
        self.handfgrpcc_uninstanced = handpkg.newHandFgrpcc()
        self.handname = handpkg.getHandName()
        # gripcontactpairs_precc is the gripcontactpairs ([[p0,p1,p2],[p0',p1',p2']] pairs) after precc (collision free)
        # gripcontactpairnormals_precc is the gripcontactpairnormals ([[n0,n1,n2],[n0',n1',n2']] pairs) after precc
        # likewise, gripcontactpairfacets_precc is the [faceid0, faceid1] pair corresponding to the upper two
        self.gripcontactpairs_precc = None
        self.gripcontactpairnormals_precc = None
        self.gripcontactpairfacets_precc = None
        self.gripcontactpairangles_precc = None
        # fliphand
        self.flipgripcontactpairs_precc = None
        self.flipgripcontactpairnormals_precc = None
        self.flipgripcontactpairfacets_precc = None
        self.flipgripcontactpairangles_precc = None

        # the final results: gripcontacts: a list of [cct0, cct1]
        # griprotmats: a list of Mat4
        # gripcontactnormals: a list of [nrml0, nrml1]
        self.gripcontacts = None
        self.griprotmats = None
        self.gripjawwidth = None
        self.gripcontactnormals = None

        self.bulletworld = BulletWorld()
        # prepare the model for collision detection
        self.objgeom = pg.packpandageom(self.objtrimesh.vertices, self.objtrimesh.face_normals, self.objtrimesh.faces)
        self.objmeshbullnode = cd.genCollisionMeshGeom(self.objgeom)
        self.bulletworld.attachRigidBody(self.objmeshbullnode)

        # for plot
        self.handplotlist = []

        # for dbupdate
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

    # def loadRtq85Models(self):
    #     """
    #     load the rtq85 model and its fgrprecc model
    #     :return:
    #     """
    #     self.rtq85hnd = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, .1])
    #     handfgrpccpath = Filename.fromOsSpecific(os.path.join(this_dir, "robotiq85/rtq85egg", "robotiq_85_tip_precc.egg"))
    #     self.handfgrpcc_uninstanced = loader.loadModel(handfgrpccpath)

    def removeHndcc(self, base, discretesize=8):
        """
        Handcc means hand collision detection

        :param discretesize: the number of hand orientations
        :return:

        author: weiwei
        date: 20161212, tsukuba
        """

        self.gripcontacts = []
        self.griprotmats = []
        self.gripjawwidth = []
        self.gripcontactnormals = []

        plotoffsetfp = 6
        self.counter = 0

        while self.counter < self.facetpairs.shape[0]:
            print str(self.counter) + "/" + str(self.facetpairs.shape[0]-1)
            # print self.gripcontactpairs_precc

            facetpair = self.facetpairs[self.counter]
            facetidx0 = facetpair[0]
            facetidx1 = facetpair[1]

            # cct0 first
            for j, contactpair in enumerate(self.gripcontactpairs_precc[self.counter]):
                cctpnt0 = contactpair[0] + plotoffsetfp * self.facetnormals[facetidx0]
                cctpnt1 = contactpair[1] + plotoffsetfp * self.facetnormals[facetidx1]
                cctnormal0 = self.gripcontactpairnormals_precc[self.counter][j][0]
                cctnormal1 = [-cctnormal0[0], -cctnormal0[1], -cctnormal0[2]]
                tmphand = self.hand
                # save initial hand pose
                initmat = tmphand.getMat()
                # save initial hand pose
                fgrcenter = (cctpnt0+cctpnt1)/2.0
                fgrdist = np.linalg.norm((cctpnt0 - cctpnt1))+2.0*plotoffsetfp
                if fgrdist > self.hand.jawwidthopen:
                    continue
                rotangle = self.gripcontactpairangles_precc[self.counter][j]
                # for rotangle in self.gripcontactpairangles_precc[self.counter]:
                tmphand.gripAt(fgrcenter[0], fgrcenter[1], fgrcenter[2], cctnormal0[0], cctnormal0[1], cctnormal0[2], rotangle, fgrdist)

                # collision detection

                self.hndbullnode = cd.genCollisionMeshMultiNp(tmphand.handnp, base.render)
                result = self.bulletworld.contactTest(self.hndbullnode)

                if not result.getNumContacts():
                    print "add handcc"
                    self.gripcontacts.append(contactpair)
                    self.griprotmats.append(tmphand.getMat())
                    self.gripjawwidth.append(fgrdist)
                    self.gripcontactnormals.append(self.gripcontactpairnormals_precc[self.counter][j])
                    # pg.plotDumbbell(base.render, (cctpnt0+cctpnt1)/2, cctcenter, length=245, thickness=5, rgba=[.4,.4,.4,1])
                    # pg.plotAxisSelf(base.render, (cctpnt0+cctpnt1)/2+245*np.array([axx[0], axx[1], axx[2]]),
                    #                 tmprtq85.getMat(), length=30, thickness=2)

                # reset initial hand pose
                tmphand.setMat(pandanpmat4 = initmat)
            # cct1 first
            for j, contactpair in enumerate(self.flipgripcontactpairs_precc[self.counter]):
                cctpnt1 = contactpair[0] + plotoffsetfp * self.facetnormals[facetidx0]
                cctpnt0 = contactpair[1] + plotoffsetfp * self.facetnormals[facetidx1]
                cctnormal1 = self.flipgripcontactpairnormals_precc[self.counter][j][0]
                cctnormal0 = [-cctnormal1[0], -cctnormal1[1], -cctnormal1[2]]
                tmphand = self.hand
                # save initial hand pose
                initmat = tmphand.getMat()
                # save initial hand pose
                fgrcenter = (cctpnt0+cctpnt1)/2
                fgrdist = np.linalg.norm((cctpnt0 - cctpnt1))+2*plotoffsetfp
                if fgrdist > self.hand.jawwidthopen:
                    continue
                rotangle = self.flipgripcontactpairangles_precc[self.counter][j]
                tmphand.gripAt(fgrcenter[0], fgrcenter[1], fgrcenter[2], cctnormal0[0], cctnormal0[1], cctnormal0[2], rotangle, fgrdist)

                # collision detection

                self.hndbullnode = cd.genCollisionMeshMultiNp(tmphand.handnp, base.render)
                result = self.bulletworld.contactTest(self.hndbullnode)

                if not result.getNumContacts():
                    print "add handcc"
                    self.gripcontacts.append(contactpair)
                    self.griprotmats.append(tmphand.getMat())
                    self.gripjawwidth.append(fgrdist)
                    self.gripcontactnormals.append(self.flipgripcontactpairnormals_precc[self.counter][j])
                    # pg.plotDumbbell(base.render, (cctpnt0+cctpnt1)/2, cctcenter, length=245, thickness=5, rgba=[.4,.4,.4,1])
                    # pg.plotAxisSelf(base.render, (cctpnt0+cctpnt1)/2+245*np.array([axx[0], axx[1], axx[2]]),
                    #                 tmprtq85.getMat(), length=30, thickness=2)

                # reset initial hand pose
                tmphand.setMat(pandanpmat4 = initmat)
            self.counter+=1
        self.counter = 0

    def removeFgrpcc(self, base, discretesize=8):
        """
        Fgrpcc means finger pre collision detection

        :return:

        author: weiwei
        date: 20161212, tsukuba
        """

        # cct0 first
        self.gripcontactpairs_precc = []
        self.gripcontactpairnormals_precc = []
        self.gripcontactpairfacets_precc = []
        self.gripcontactpairangles_precc = []
        # cct1 first
        self.flipgripcontactpairs_precc = []
        self.flipgripcontactpairnormals_precc = []
        self.flipgripcontactpairfacets_precc = []
        self.flipgripcontactpairangles_precc = []

        plotoffsetfp = 6
        contactdifference = 5

        self.counter = 0

        bulletworldray = BulletWorld()
        bulletworldray.attachRigidBody(self.objmeshbullnode)
        while self.counter < self.facetpairs.shape[0]:
            print str(self.counter) + "/" + str(self.facetpairs.shape[0]-1)
            # cct0 first
            self.gripcontactpairs_precc.append([])
            self.gripcontactpairnormals_precc.append([])
            self.gripcontactpairfacets_precc.append([])
            self.gripcontactpairangles_precc.append([])
            # cct1 first
            self.flipgripcontactpairs_precc.append([])
            self.flipgripcontactpairnormals_precc.append([])
            self.flipgripcontactpairfacets_precc.append([])
            self.flipgripcontactpairangles_precc.append([])

            facetpair = self.facetpairs[self.counter]
            facetidx0 = facetpair[0]
            facetidx1 = facetpair[1]

            # cctpnt0 first
            for j, contactpair in enumerate(self.gripcontactpairs[self.counter]):
                cctpnt0 = contactpair[0] + plotoffsetfp * self.facetnormals[facetidx0]
                cctpnt1 = contactpair[1] + plotoffsetfp * self.facetnormals[facetidx1]
                cctnormal0 = self.facetnormals[facetidx0]
                cctnormal1 = [-cctnormal0[0], -cctnormal0[1], -cctnormal0[2]]
                tmphand = self.hand
                # save initial hand pose
                fgrcenter = (cctpnt0+cctpnt1)/2
                fgrdist = np.linalg.norm((cctpnt0 - cctpnt1))+2*plotoffsetfp
                if fgrdist > self.hand.jawwidthopen:
                    continue
                for angleid in range(discretesize):
                    rotangle = 360.0 / float(discretesize) * float(angleid)
                    tmphand.gripAt(fgrcenter[0], fgrcenter[1], fgrcenter[2], cctnormal0[0], cctnormal0[1], cctnormal0[2], rotangle, fgrdist)

                    #dist of cctpnt1 from surface
                    pFrom = Point3(cctpnt1[0], cctpnt1[1], cctpnt1[2])
                    pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2])*9999
                    result1 = bulletworldray.rayTestClosest(pFrom, pTo)
                    # distances of virtual contact points
                    cctpnt100 = pg.v3ToNp(pg.npToV3(cctpnt1)+tmphand.handpccnp.getMat().getRow3(2)*15)
                    cctpnt110 = pg.v3ToNp(pg.npToV3(cctpnt1)-tmphand.handpccnp.getMat().getRow3(2)*15)
                    cctpnt101 = pg.v3ToNp(pg.npToV3(cctpnt1)+tmphand.handpccnp.getMat().getRow3(2)*30)
                    cctpnt111 = pg.v3ToNp(pg.npToV3(cctpnt1)-tmphand.handpccnp.getMat().getRow3(2)*30)
                    #100
                    pFrom = Point3(cctpnt100[0], cctpnt100[1], cctpnt100[2])
                    pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2])*9999
                    result100 = bulletworldray.rayTestClosest(pFrom, pTo)
                    #110
                    pFrom = Point3(cctpnt110[0], cctpnt110[1], cctpnt110[2])
                    pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2])*9999
                    result110 = bulletworldray.rayTestClosest(pFrom, pTo)
                    #101
                    pFrom = Point3(cctpnt101[0], cctpnt101[1], cctpnt101[2])
                    pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2])*9999
                    result101 = bulletworldray.rayTestClosest(pFrom, pTo)
                    #111
                    pFrom = Point3(cctpnt111[0], cctpnt111[1], cctpnt111[2])
                    pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2])*9999
                    result111 = bulletworldray.rayTestClosest(pFrom, pTo)

                    if not result1.hasHit():
                        continue
                    if ((not result100.hasHit()) and (not result101.hasHit())) or\
                            ((not result110.hasHit()) and (not result111.hasHit())):
                        continue
                    distresult1 = (result1.getHitPos()-pg.npToV3(cctpnt1)).length()
                    if result100.hasHit():
                        distresult = (result100.getHitPos()-pg.npToV3(cctpnt100)).length()
                        if distresult-distresult1 > contactdifference:
                            continue
                    if result101.hasHit():
                        distresult = (result101.getHitPos()-pg.npToV3(cctpnt101)).length()
                        if distresult-distresult1 > contactdifference:
                            continue
                    if result110.hasHit():
                        distresult = (result110.getHitPos()-pg.npToV3(cctpnt110)).length()
                        if distresult-distresult1 > contactdifference:
                            continue
                    if result111.hasHit():
                        distresult = (result111.getHitPos()-pg.npToV3(cctpnt111)).length()
                        if distresult-distresult1 > contactdifference:
                            continue

                    # prepare the model for collision detection
                    handpccbullnode = cd.genCollisionMeshMultiNp(tmphand.handpccnp)
                    result = self.bulletworld.contactTest(handpccbullnode)

                    if not result.getNumContacts():
                        print "add precc"
                        self.gripcontactpairs_precc[-1].append(contactpair)
                        self.gripcontactpairnormals_precc[-1].append(self.gripcontactpairnormals[self.counter][j])
                        self.gripcontactpairfacets_precc[-1].append(self.gripcontactpairfacets[self.counter])
                        self.gripcontactpairangles_precc[-1].append(rotangle)
            # cctpnt1 first
            for j, contactpair in enumerate(self.gripcontactpairs[self.counter]):
                cctpnt1 = contactpair[0] + plotoffsetfp * self.facetnormals[facetidx0]
                cctpnt0 = contactpair[1] + plotoffsetfp * self.facetnormals[facetidx1]
                cctnormal1 = self.facetnormals[facetidx0]
                cctnormal0 = [-cctnormal1[0], -cctnormal1[1], -cctnormal1[2]]
                tmphand = self.hand
                # save initial hand pose
                fgrcenter = (cctpnt0 + cctpnt1) / 2
                fgrdist = np.linalg.norm((cctpnt0 - cctpnt1)) + 2 * plotoffsetfp
                if fgrdist > self.hand.jawwidthopen:
                    continue
                for angleid in range(discretesize):
                    rotangle = 360.0 / float(discretesize) * float(angleid)
                    tmphand.gripAt(fgrcenter[0], fgrcenter[1], fgrcenter[2], cctnormal0[0], cctnormal0[1],
                                   cctnormal0[2], rotangle, fgrdist)

                    # dist of cctpnt1 from surface
                    pFrom = Point3(cctpnt1[0], cctpnt1[1], cctpnt1[2])
                    pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                    result1 = bulletworldray.rayTestClosest(pFrom, pTo)
                    # distances of virtual contact points
                    cctpnt100 = pg.v3ToNp(pg.npToV3(cctpnt1) + tmphand.handpccnp.getMat().getRow3(2) * 15)
                    cctpnt110 = pg.v3ToNp(pg.npToV3(cctpnt1) - tmphand.handpccnp.getMat().getRow3(2) * 15)
                    cctpnt101 = pg.v3ToNp(pg.npToV3(cctpnt1) + tmphand.handpccnp.getMat().getRow3(2) * 30)
                    cctpnt111 = pg.v3ToNp(pg.npToV3(cctpnt1) - tmphand.handpccnp.getMat().getRow3(2) * 30)
                    # 100
                    pFrom = Point3(cctpnt100[0], cctpnt100[1], cctpnt100[2])
                    pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                    result100 = bulletworldray.rayTestClosest(pFrom, pTo)
                    # 110
                    pFrom = Point3(cctpnt110[0], cctpnt110[1], cctpnt110[2])
                    pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                    result110 = bulletworldray.rayTestClosest(pFrom, pTo)
                    # 101
                    pFrom = Point3(cctpnt101[0], cctpnt101[1], cctpnt101[2])
                    pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                    result101 = bulletworldray.rayTestClosest(pFrom, pTo)
                    # 111
                    pFrom = Point3(cctpnt111[0], cctpnt111[1], cctpnt111[2])
                    pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                    result111 = bulletworldray.rayTestClosest(pFrom, pTo)

                    if not result1.hasHit():
                        continue
                    if ((not result100.hasHit()) and (not result101.hasHit())) or \
                            ((not result110.hasHit()) and (not result111.hasHit())):
                        continue
                    distresult1 = (result1.getHitPos() - pg.npToV3(cctpnt1)).length()
                    if result100.hasHit():
                        distresult = (result100.getHitPos() - pg.npToV3(cctpnt100)).length()
                        if distresult - distresult1 > contactdifference:
                            continue
                    if result101.hasHit():
                        distresult = (result101.getHitPos() - pg.npToV3(cctpnt101)).length()
                        if distresult - distresult1 > contactdifference:
                            continue
                    if result110.hasHit():
                        distresult = (result110.getHitPos() - pg.npToV3(cctpnt110)).length()
                        if distresult - distresult1 > contactdifference:
                            continue
                    if result111.hasHit():
                        distresult = (result111.getHitPos() - pg.npToV3(cctpnt111)).length()
                        if distresult - distresult1 > contactdifference:
                            continue

                    # prepare the model for collision detection
                    handpccbullnode = cd.genCollisionMeshMultiNp(tmphand.handpccnp)
                    result = self.bulletworld.contactTest(handpccbullnode)

                    if not result.getNumContacts():
                        print "add precc"
                        self.flipgripcontactpairs_precc[-1].append(contactpair)
                        self.flipgripcontactpairnormals_precc[-1].append(
                            self.gripcontactpairnormals[self.counter][j])
                        self.flipgripcontactpairfacets_precc[-1].append(
                            self.gripcontactpairfacets[self.counter])
                        self.flipgripcontactpairangles_precc[-1].append(rotangle)
            self.counter += 1
        self.counter=0

    def saveToDB(self, gdb):
        """
        save the result to mysqldatabase

        :param gdb: is an object of the GraspDB class in the database package
        :return:

        author: weiwei
        date: 20170110
        """

        # save to database
        gdb = db.GraspDB()

        idhand = gdb.loadIdHand(self.handname)

        sql = "SELECT * FROM freeairgrip, object WHERE freeairgrip.idobject = object.idobject AND \
                object.name LIKE '%s' AND freeairgrip.idhand LIKE '%s'" % (self.dbobjname, idhand)
        result = gdb.execute(sql)
        if not result:
            sql = "SELECT idobject FROM object WHERE name LIKE '%s'" % self.dbobjname
            returnlist = gdb.execute(sql)
            if len(returnlist) != 0:
                idobject = returnlist[0][0]
            else:
                sql = "INSERT INTO object(name) VALUES('%s')" % self.dbobjname
                idobject = gdb.execute(sql)
            print self.gripcontacts
            for i in range(len(self.gripcontacts)):
                sql = "INSERT INTO freeairgrip(idobject, contactpnt0, contactpnt1, \
                        contactnormal0, contactnormal1, rotmat, jawwidth, idhand) \
                       VALUES('%s', '%s', '%s', '%s', '%s', '%s', '%s', %d)" % \
                      (idobject, dc.v3ToStr(self.gripcontacts[i][0]), dc.v3ToStr(self.gripcontacts[i][1]),
                       dc.v3ToStr(self.gripcontactnormals[i][0]), dc.v3ToStr(self.gripcontactnormals[i][1]),
                       dc.mat4ToStr(self.griprotmats[i]), str(self.gripjawwidth[i]), idhand)
                gdb.execute(sql)
        else:
            print "Grasps already saved or duplicated filename!"

    def removeFgrpccShow(self, base, discretesize = 8):
        """
        Fgrpcc means finger pre collision detection
        This one is specially written for demonstration

        :return:

        author: weiwei
        date: 20161201, osaka
        """

        # 6 is used because I am supposing 4+2 where 4 is the default
        # margin of bullet in panda3d. (NOTE: This is a guess)
        plotoffsetfp = 0
        contactdist = 6
        contactdifference = 3

        npbrchild = base.render.find("**/tempplot")
        if npbrchild:
            npbrchild.removeNode()

        # for fast delete
        brchild = NodePath('tempplot')
        brchild.reparentTo(base.render)

        self.counter += 1
        if self.counter >= self.facetpairs.shape[0]:
            self.counter = 0

        facetpair = self.facetpairs[self.counter]
        facetidx0 = facetpair[0]
        facetidx1 = facetpair[1]
        geomfacet0 = pg.packpandageom(self.objtrimesh.vertices+
                                       np.tile(plotoffsetfp*self.facetnormals[facetidx0],
                                               [self.objtrimesh.vertices.shape[0],1]),
                                       self.objtrimesh.face_normals[self.facets[facetidx0]],
                                       self.objtrimesh.faces[self.facets[facetidx0]])
        geomfacet1 = pg.packpandageom(self.objtrimesh.vertices+
                                       np.tile(plotoffsetfp*self.facetnormals[facetidx1],
                                               [self.objtrimesh.vertices.shape[0],1]),
                                       self.objtrimesh.face_normals[self.facets[facetidx1]],
                                       self.objtrimesh.faces[self.facets[facetidx1]])
        # show the facetpair
        node0 = GeomNode('pair0')
        node0.addGeom(geomfacet0)
        star0 = NodePath('pair0')
        star0.attachNewNode(node0)
        facetcolorarray = self.facetcolorarray
        star0.setColor(Vec4(facetcolorarray[facetidx0][0], facetcolorarray[facetidx0][1],
                           facetcolorarray[facetidx0][2], facetcolorarray[facetidx0][3]))
        star0.setTwoSided(True)
        star0.reparentTo(brchild)
        node1 = GeomNode('pair1')
        node1.addGeom(geomfacet1)
        star1 = NodePath('pair1')
        star1.attachNewNode(node1)
        star1.setColor(Vec4(facetcolorarray[facetidx1][0], facetcolorarray[facetidx1][1],
                           facetcolorarray[facetidx1][2], facetcolorarray[facetidx1][3]))
        star1.setTwoSided(True)
        star1.reparentTo(brchild)

        bulletworldray = BulletWorld()
        bulletworldray.attachRigidBody(self.objmeshbullnode)

        # cct0 first
        for j, contactpair in enumerate(self.gripcontactpairs[self.counter]):
            cctpnt0 = contactpair[0] + contactdist * self.facetnormals[facetidx0]
            cctpnt1 = contactpair[1] + contactdist * self.facetnormals[facetidx1]
            cctnormal0 = self.facetnormals[facetidx0]
            cctnormal1 = [-cctnormal0[0], -cctnormal0[1], -cctnormal0[2]]
            # save initial hand pose
            fgrcenter = (cctpnt0 + cctpnt1) / 2
            fgrdist = np.linalg.norm((cctpnt0 - cctpnt1)) + 2 * contactdist
            if fgrdist > self.hand.jawwidthopen:
                continue
            for angleid in range(discretesize):
                tmphand = self.handpkg.newHandNM()
                rotangle = 360.0 / float(discretesize) * float(angleid)
                tmphand.gripAt(fgrcenter[0], fgrcenter[1], fgrcenter[2], cctnormal0[0], cctnormal0[1], cctnormal0[2],
                               rotangle, fgrdist)

                # dist of cctpnt1 from surface
                pFrom = Point3(cctpnt1[0], cctpnt1[1], cctpnt1[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result1 = bulletworldray.rayTestClosest(pFrom, pTo)
                # distances of virtual contact points
                cctpnt100 = pg.v3ToNp(pg.npToV3(cctpnt1) + tmphand.handpccnp.getMat().getRow3(2) * 15)
                cctpnt110 = pg.v3ToNp(pg.npToV3(cctpnt1) - tmphand.handpccnp.getMat().getRow3(2) * 15)
                cctpnt101 = pg.v3ToNp(pg.npToV3(cctpnt1) + tmphand.handpccnp.getMat().getRow3(2) * 30)
                cctpnt111 = pg.v3ToNp(pg.npToV3(cctpnt1) - tmphand.handpccnp.getMat().getRow3(2) * 30)
                # 100
                pFrom = Point3(cctpnt100[0], cctpnt100[1], cctpnt100[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result100 = bulletworldray.rayTestClosest(pFrom, pTo)
                # 110
                pFrom = Point3(cctpnt110[0], cctpnt110[1], cctpnt110[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result110 = bulletworldray.rayTestClosest(pFrom, pTo)
                # 101
                pFrom = Point3(cctpnt101[0], cctpnt101[1], cctpnt101[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result101 = bulletworldray.rayTestClosest(pFrom, pTo)
                # 111
                pFrom = Point3(cctpnt111[0], cctpnt111[1], cctpnt111[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result111 = bulletworldray.rayTestClosest(pFrom, pTo)

                if not result1.hasHit():
                    continue
                if ((not result100.hasHit()) and (not result101.hasHit())) or \
                        ((not result110.hasHit()) and (not result111.hasHit())):
                    tmphand.handpccnp.setColor(.5, 0, 0, .3)
                    tmphand.handpccnp.reparentTo(brchild)
                    continue
                distresult1 = (result1.getHitPos() - pg.npToV3(cctpnt1)).length()
                if result100.hasHit():
                    distresult = (result100.getHitPos() - pg.npToV3(cctpnt100)).length()
                    if distresult - distresult1 > contactdifference:
                        tmphand.handpccnp.setColor(.5, 0, 0, .3)
                        tmphand.handpccnp.reparentTo(brchild)
                        continue
                if result101.hasHit():
                    distresult = (result101.getHitPos() - pg.npToV3(cctpnt101)).length()
                    if distresult - distresult1 > contactdifference:
                        tmphand.handpccnp.setColor(.5, 0, 0, .3)
                        tmphand.handpccnp.reparentTo(brchild)
                        continue
                if result110.hasHit():
                    distresult = (result110.getHitPos() - pg.npToV3(cctpnt110)).length()
                    if distresult - distresult1 > contactdifference:
                        tmphand.handpccnp.setColor(.5, 0, 0, .3)
                        tmphand.handpccnp.reparentTo(brchild)
                        continue
                if result111.hasHit():
                    distresult = (result111.getHitPos() - pg.npToV3(cctpnt111)).length()
                    if distresult - distresult1 > contactdifference:
                        tmphand.handpccnp.setColor(.5, 0, 0, .3)
                        tmphand.handpccnp.reparentTo(brchild)
                        continue

                # prepare the model for collision detection
                handpccbullnode = cd.genCollisionMeshMultiNp(tmphand.handpccnp, brchild)
                result = self.bulletworld.contactTest(handpccbullnode)

                # for contact in result.getContacts():
                #     cp = contact.getManifoldPoint()
                #     pg.plotSphere(brchild, pos=cp.getLocalPointA(), radius=3, rgba=Vec4(1, 0, 0, 1))
                #     pg.plotSphere(brchild, pos=cp.getLocalPointB(), radius=3, rgba=Vec4(0, 0, 1, 1))

                if result.getNumContacts():
                    tmphand.handpccnp.setColor(.5, 0, 0, .3)
                else:
                    tmphand.handpccnp.setColor(1, 1, 1, .3)
                tmphand.handpccnp.reparentTo(brchild)

                pg.plotArrow(star0, spos=cctpnt0,
                                epos=cctpnt0 + plotoffsetfp*self.facetnormals[facetidx0] + cctnormal0,
                                rgba=[facetcolorarray[facetidx0][0], facetcolorarray[facetidx0][1],
                                      facetcolorarray[facetidx0][2], facetcolorarray[facetidx0][3]], length=10)
                pg.plotArrow(star1, spos=cctpnt1,
                                epos=cctpnt1 + plotoffsetfp*self.facetnormals[facetidx1] + cctnormal1,
                                rgba=[facetcolorarray[facetidx1][0], facetcolorarray[facetidx1][1],
                                      facetcolorarray[facetidx1][2], facetcolorarray[facetidx1][3]], length=10)
        # cct1 first
        for j, contactpair in enumerate(self.gripcontactpairs[self.counter]):
            # swap cct
            cctpnt1 = contactpair[0] + contactdist * self.facetnormals[facetidx0]
            cctpnt0 = contactpair[1] + contactdist * self.facetnormals[facetidx1]
            cctnormal1 = self.facetnormals[facetidx0]
            cctnormal0 = [-cctnormal1[0], -cctnormal1[1], -cctnormal1[2]]
            # save initial hand pose
            fgrcenter = (cctpnt0 + cctpnt1) / 2
            fgrdist = np.linalg.norm((cctpnt0 - cctpnt1)) + 2 * contactdist
            if fgrdist > self.hand.jawwidthopen:
                continue
            for angleid in range(discretesize):
                tmphand = self.handpkg.newHandNM()
                rotangle = 360.0 / float(discretesize) * float(angleid)
                tmphand.gripAt(fgrcenter[0], fgrcenter[1], fgrcenter[2], cctnormal0[0], cctnormal0[1], cctnormal0[2],
                               rotangle, fgrdist)

                # dist of cctpnt1 from surface
                pFrom = Point3(cctpnt1[0], cctpnt1[1], cctpnt1[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result1 = bulletworldray.rayTestClosest(pFrom, pTo)
                # distances of virtual contact points
                cctpnt100 = pg.v3ToNp(pg.npToV3(cctpnt1) + tmphand.handpccnp.getMat().getRow3(2) * 15)
                cctpnt110 = pg.v3ToNp(pg.npToV3(cctpnt1) - tmphand.handpccnp.getMat().getRow3(2) * 15)
                cctpnt101 = pg.v3ToNp(pg.npToV3(cctpnt1) + tmphand.handpccnp.getMat().getRow3(2) * 30)
                cctpnt111 = pg.v3ToNp(pg.npToV3(cctpnt1) - tmphand.handpccnp.getMat().getRow3(2) * 30)
                # 100
                pFrom = Point3(cctpnt100[0], cctpnt100[1], cctpnt100[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result100 = bulletworldray.rayTestClosest(pFrom, pTo)
                # 110
                pFrom = Point3(cctpnt110[0], cctpnt110[1], cctpnt110[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result110 = bulletworldray.rayTestClosest(pFrom, pTo)
                # 101
                pFrom = Point3(cctpnt101[0], cctpnt101[1], cctpnt101[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result101 = bulletworldray.rayTestClosest(pFrom, pTo)
                # 111
                pFrom = Point3(cctpnt111[0], cctpnt111[1], cctpnt111[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result111 = bulletworldray.rayTestClosest(pFrom, pTo)

                if not result1.hasHit():
                    continue
                if ((not result100.hasHit()) and (not result101.hasHit())) or \
                        ((not result110.hasHit()) and (not result111.hasHit())):
                    tmphand.handpccnp.setColor(.5, 0, 0, .3)
                    tmphand.handpccnp.reparentTo(brchild)
                    continue
                distresult1 = (result1.getHitPos() - pg.npToV3(cctpnt1)).length()
                if result100.hasHit():
                    distresult = (result100.getHitPos() - pg.npToV3(cctpnt100)).length()
                    if distresult - distresult1 > contactdifference:
                        tmphand.handpccnp.setColor(.5, 0, 0, .3)
                        tmphand.handpccnp.reparentTo(brchild)
                        continue
                if result101.hasHit():
                    distresult = (result101.getHitPos() - pg.npToV3(cctpnt101)).length()
                    if distresult - distresult1 > contactdifference:
                        tmphand.handpccnp.setColor(.5, 0, 0, .3)
                        tmphand.handpccnp.reparentTo(brchild)
                        continue
                if result110.hasHit():
                    distresult = (result110.getHitPos() - pg.npToV3(cctpnt110)).length()
                    if distresult - distresult1 > contactdifference:
                        tmphand.handpccnp.setColor(.5, 0, 0, .3)
                        tmphand.handpccnp.reparentTo(brchild)
                        continue
                if result111.hasHit():
                    distresult = (result111.getHitPos() - pg.npToV3(cctpnt111)).length()
                    if distresult - distresult1 > contactdifference:
                        tmphand.handpccnp.setColor(.5, 0, 0, .3)
                        tmphand.handpccnp.reparentTo(brchild)
                        continue

                # prepare the model for collision detection
                handpccbullnode = cd.genCollisionMeshMultiNp(tmphand.handpccnp, brchild)
                result = self.bulletworld.contactTest(handpccbullnode)

                # for contact in result.getContacts():
                #     cp = contact.getManifoldPoint()
                #     pg.plotSphere(brchild, pos=cp.getLocalPointA(), radius=3, rgba=Vec4(1, 0, 0, 1))
                #     pg.plotSphere(brchild, pos=cp.getLocalPointB(), radius=3, rgba=Vec4(0, 0, 1, 1))

                if result.getNumContacts():
                    tmphand.handpccnp.setColor(.5, 0, 0, .3)
                else:
                    tmphand.handpccnp.setColor(1, 1, 1, .3)
                tmphand.handpccnp.reparentTo(brchild)

                pg.plotArrow(star0, spos=cctpnt0,
                                epos=cctpnt0 + plotoffsetfp*self.facetnormals[facetidx0] + cctnormal0,
                                rgba=[facetcolorarray[facetidx0][0], facetcolorarray[facetidx0][1],
                                      facetcolorarray[facetidx0][2], facetcolorarray[facetidx0][3]], length=10)
                pg.plotArrow(star1, spos=cctpnt1,
                                epos=cctpnt1 + plotoffsetfp*self.facetnormals[facetidx1] + cctnormal1,
                                rgba=[facetcolorarray[facetidx1][0], facetcolorarray[facetidx1][1],
                                      facetcolorarray[facetidx1][2], facetcolorarray[facetidx1][3]], length=10)

    def removeFgrpccShowLeft(self, base, discretesize = 8):
        """
        Fgrpcc means finger pre collision detection
        This one is specially written for demonstration
        Plot the available grips

        :return:

        author: weiwei
        date: 20161212, tsukuba
        """

        plotoffsetfp = 6
        contactdist = 6
        contactdifference = 3

        self.counter += 1
        if self.counter >= self.facetpairs.shape[0]:
            return
        else:
            print str(self.counter) + "/" + str(self.facetpairs.shape[0]-1)

            facetpair = self.facetpairs[self.counter]
            facetidx0 = facetpair[0]
            facetidx1 = facetpair[1]

        bulletworldray = BulletWorld()
        bulletworldray.attachRigidBody(self.objmeshbullnode)

        # cct0 first
        for j, contactpair in enumerate(self.gripcontactpairs[self.counter]):
            cctpnt0 = contactpair[0] + contactdist * self.facetnormals[facetidx0]
            cctpnt1 = contactpair[1] + contactdist * self.facetnormals[facetidx1]
            cctnormal0 = self.facetnormals[facetidx0]
            cctnormal1 = [-cctnormal0[0], -cctnormal0[1], -cctnormal0[2]]
            # save initial hand pose
            fgrcenter = (cctpnt0 + cctpnt1) / 2
            fgrdist = np.linalg.norm((cctpnt0 - cctpnt1)) + 2 * contactdist
            if fgrdist > self.hand.jawwidthopen:
                continue
            for angleid in range(discretesize):
                tmphand = self.handpkg.newHandNM()
                rotangle = 360.0 / float(discretesize) * float(angleid)
                tmphand.gripAt(fgrcenter[0], fgrcenter[1], fgrcenter[2], cctnormal0[0], cctnormal0[1], cctnormal0[2],
                               rotangle, fgrdist)

                # dist of cctpnt1 from surface
                pFrom = Point3(cctpnt1[0], cctpnt1[1], cctpnt1[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result1 = bulletworldray.rayTestClosest(pFrom, pTo)
                # distances of virtual contact points
                cctpnt100 = pg.v3ToNp(pg.npToV3(cctpnt1) + tmphand.handpccnp.getMat().getRow3(2) * 15)
                cctpnt110 = pg.v3ToNp(pg.npToV3(cctpnt1) - tmphand.handpccnp.getMat().getRow3(2) * 15)
                cctpnt101 = pg.v3ToNp(pg.npToV3(cctpnt1) + tmphand.handpccnp.getMat().getRow3(2) * 30)
                cctpnt111 = pg.v3ToNp(pg.npToV3(cctpnt1) - tmphand.handpccnp.getMat().getRow3(2) * 30)
                # 100
                pFrom = Point3(cctpnt100[0], cctpnt100[1], cctpnt100[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result100 = bulletworldray.rayTestClosest(pFrom, pTo)
                # 110
                pFrom = Point3(cctpnt110[0], cctpnt110[1], cctpnt110[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result110 = bulletworldray.rayTestClosest(pFrom, pTo)
                # 101
                pFrom = Point3(cctpnt101[0], cctpnt101[1], cctpnt101[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result101 = bulletworldray.rayTestClosest(pFrom, pTo)
                # 111
                pFrom = Point3(cctpnt111[0], cctpnt111[1], cctpnt111[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result111 = bulletworldray.rayTestClosest(pFrom, pTo)

                if not result1.hasHit():
                    continue
                if ((not result100.hasHit()) and (not result101.hasHit())) or \
                        ((not result110.hasHit()) and (not result111.hasHit())):
                    continue
                distresult1 = (result1.getHitPos() - pg.npToV3(cctpnt1)).length()
                if result100.hasHit():
                    distresult = (result100.getHitPos() - pg.npToV3(cctpnt100)).length()
                    if distresult - distresult1 > contactdifference:
                        continue
                if result101.hasHit():
                    distresult = (result101.getHitPos() - pg.npToV3(cctpnt101)).length()
                    if distresult - distresult1 > contactdifference:
                        continue
                if result110.hasHit():
                    distresult = (result110.getHitPos() - pg.npToV3(cctpnt110)).length()
                    if distresult - distresult1 > contactdifference:
                        continue
                if result111.hasHit():
                    distresult = (result111.getHitPos() - pg.npToV3(cctpnt111)).length()
                    if distresult - distresult1 > contactdifference:
                        continue

                # prepare the model for collision detection
                handpccbullnode = cd.genCollisionMeshMultiNp(tmphand.handpccnp)
                result = self.bulletworld.contactTest(handpccbullnode)

                if not result.getNumContacts():
                    tmphand.handpccnp.setColor(1, 1, 1, .3)
                    tmphand.handpccnp.reparentTo(base.render)

        # cct1 first
        for j, contactpair in enumerate(self.gripcontactpairs[self.counter]):
            cctpnt1 = contactpair[0] + contactdist * self.facetnormals[facetidx0]
            cctpnt0 = contactpair[1] + contactdist * self.facetnormals[facetidx1]
            cctnormal1 = self.facetnormals[facetidx0]
            cctnormal0 = [-cctnormal0[0], -cctnormal0[1], -cctnormal0[2]]
            # save initial hand pose
            fgrcenter = (cctpnt0 + cctpnt1) / 2
            fgrdist = np.linalg.norm((cctpnt0 - cctpnt1)) + 2 * contactdist
            if fgrdist > self.hand.jawwidthopen:
                continue
            for angleid in range(discretesize):
                tmphand = self.handpkg.newHandNM()
                rotangle = 360.0 / float(discretesize) * float(angleid)
                tmphand.gripAt(fgrcenter[0], fgrcenter[1], fgrcenter[2], cctnormal0[0], cctnormal0[1], cctnormal0[2],
                               rotangle, fgrdist)

                # dist of cctpnt1 from surface
                pFrom = Point3(cctpnt1[0], cctpnt1[1], cctpnt1[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result1 = bulletworldray.rayTestClosest(pFrom, pTo)
                # distances of virtual contact points
                cctpnt100 = pg.v3ToNp(pg.npToV3(cctpnt1) + tmphand.handpccnp.getMat().getRow3(2) * 15)
                cctpnt110 = pg.v3ToNp(pg.npToV3(cctpnt1) - tmphand.handpccnp.getMat().getRow3(2) * 15)
                cctpnt101 = pg.v3ToNp(pg.npToV3(cctpnt1) + tmphand.handpccnp.getMat().getRow3(2) * 30)
                cctpnt111 = pg.v3ToNp(pg.npToV3(cctpnt1) - tmphand.handpccnp.getMat().getRow3(2) * 30)
                # 100
                pFrom = Point3(cctpnt100[0], cctpnt100[1], cctpnt100[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result100 = bulletworldray.rayTestClosest(pFrom, pTo)
                # 110
                pFrom = Point3(cctpnt110[0], cctpnt110[1], cctpnt110[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result110 = bulletworldray.rayTestClosest(pFrom, pTo)
                # 101
                pFrom = Point3(cctpnt101[0], cctpnt101[1], cctpnt101[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result101 = bulletworldray.rayTestClosest(pFrom, pTo)
                # 111
                pFrom = Point3(cctpnt111[0], cctpnt111[1], cctpnt111[2])
                pTo = pFrom + Vec3(cctnormal0[0], cctnormal0[1], cctnormal0[2]) * 9999
                result111 = bulletworldray.rayTestClosest(pFrom, pTo)

                if not result1.hasHit():
                    continue
                if ((not result100.hasHit()) and (not result101.hasHit())) or \
                        ((not result110.hasHit()) and (not result111.hasHit())):
                    continue
                distresult1 = (result1.getHitPos() - pg.npToV3(cctpnt1)).length()
                if result100.hasHit():
                    distresult = (result100.getHitPos() - pg.npToV3(cctpnt100)).length()
                    if distresult - distresult1 > contactdifference:
                        continue
                if result101.hasHit():
                    distresult = (result101.getHitPos() - pg.npToV3(cctpnt101)).length()
                    if distresult - distresult1 > contactdifference:
                        continue
                if result110.hasHit():
                    distresult = (result110.getHitPos() - pg.npToV3(cctpnt110)).length()
                    if distresult - distresult1 > contactdifference:
                        continue
                if result111.hasHit():
                    distresult = (result111.getHitPos() - pg.npToV3(cctpnt111)).length()
                    if distresult - distresult1 > contactdifference:
                        continue

                # prepare the model for collision detection
                handpccbullnode = cd.genCollisionMeshMultiNp(tmphand.handpccnp)
                result = self.bulletworld.contactTest(handpccbullnode)

                if not result.getNumContacts():
                    tmphand.handpccnp.setColor(1, 1, 1, .3)
                    tmphand.handpccnp.reparentTo(base.render)

    def removeHndccShow(self, base, discretesize=8):
        """
        Handcc means hand collision detection
        This one is developed for demonstration
        This function should be called after executing removeHndcc

        :param discretesize: the number of hand orientations
        :return: delayTime

        author: weiwei
        date: 20161212, tsukuba
        """

        if self.handplotlist:
            for handplotlist in self.handplotlist:
                handplotlist.removeNode()
        self.handplotlist = []

        self.gripcontacts = []
        self.griprotmats = []
        self.gripjawwidth = []
        self.gripcontactnormals = []

        plotoffsetfp = 6

        self.counter += 1
        if self.counter >= self.facetpairs.shape[0]:
            self.counter = 0

        print str(self.counter) + "/" + str(self.facetpairs.shape[0]-1)

        facetpair = self.facetpairs[self.counter]
        facetidx0 = facetpair[0]
        facetidx1 = facetpair[1]

        # cct0 first
        for j, contactpair in enumerate(self.gripcontactpairs_precc[self.counter]):
            print j, contactpair
            cctpnt0 = contactpair[0] + plotoffsetfp * self.facetnormals[facetidx0]
            cctpnt1 = contactpair[1] + plotoffsetfp * self.facetnormals[facetidx1]
            cctnormal0 = self.gripcontactpairnormals_precc[self.counter][j][0]
            cctnormal1 = [-cctnormal0[0], -cctnormal0[1], -cctnormal0[2]]
            tmphand = self.handpkg.newHandNM(hndcolor=[.2,.7,.2,1])
            # save initial hand pose
            fgrcenter = (cctpnt0+cctpnt1)/2
            fgrdist = np.linalg.norm((cctpnt0 - cctpnt1))+2*plotoffsetfp
            if fgrdist > self.hand.jawwidthopen:
                continue
            rotangle = self.gripcontactpairangles_precc[self.counter][j]
            # for rotangle in self.gripcontactpairangles_precc[self.counter]:
            tmphand.gripAt(fgrcenter[0], fgrcenter[1], fgrcenter[2], cctnormal0[0], cctnormal0[1], cctnormal0[2], rotangle, fgrdist)

            # collision detection

            self.hndbullnode = cd.genCollisionMeshMultiNp(tmphand.handnp, base.render)
            result = self.bulletworld.contactTest(self.hndbullnode)

            if not result.getNumContacts():
                # pandageom.plotDumbbell(base.render, (cctpnt0+cctpnt1)/2, cctcenter, length=245, thickness=5, rgba=[.4,.4,.4,1])
                # pandageom.plotAxisSelf(base.render, (cctpnt0+cctpnt1)/2+245*np.array([axx[0], axx[1], axx[2]]),
                #                 tmprtq85.getMat(), length=30, thickness=2)
                tmphand.setColor([0, 0, 1, .3])
                tmphand.reparentTo(base.render)
                self.handplotlist.append(tmphand)
            else:
                # for contact in result.getContacts():
                    # cp = contact.getManifoldPoint()
                    # pandageom.plotSphere(brchild, pos=cp.getLocalPointA(), radius=3, rgba=Vec4(1, 0, 0, 1))
                    # pandageom.plotSphere(brchild, pos=cp.getLocalPointB(), radius=3, rgba=Vec4(0, 0, 1, 1))
                tmphand.setColor([.5, 0, 0, .3])
                tmphand.reparentTo(base.render)
                self.handplotlist.append(tmphand)

        # cct1 first
        for j, contactpair in enumerate(self.flipgripcontactpairs_precc[self.counter]):
            print j, contactpair
            cctpnt1 = contactpair[0] + plotoffsetfp * self.facetnormals[facetidx0]
            cctpnt0 = contactpair[1] + plotoffsetfp * self.facetnormals[facetidx1]
            cctnormal1 = self.flipgripcontactpairnormals_precc[self.counter][j][0]
            cctnormal0 = [-cctnormal1[0], -cctnormal1[1], -cctnormal1[2]]
            tmphand = self.handpkg.newHandNM(hndcolor=[.2,.7,.2,1])
            # save initial hand pose
            fgrcenter = (cctpnt0+cctpnt1)/2
            fgrdist = np.linalg.norm((cctpnt0 - cctpnt1))+2*plotoffsetfp
            if fgrdist > self.hand.jawwidthopen:
                continue
            rotangle = self.flipgripcontactpairangles_precc[self.counter][j]
            tmphand.gripAt(fgrcenter[0], fgrcenter[1], fgrcenter[2], cctnormal0[0], cctnormal0[1], cctnormal0[2], rotangle, fgrdist)

            # collision detection

            self.hndbullnode = cd.genCollisionMeshMultiNp(tmphand.handnp, base.render)
            result = self.bulletworld.contactTest(self.hndbullnode)

            if not result.getNumContacts():
                # pandageom.plotDumbbell(base.render, (cctpnt0+cctpnt1)/2, cctcenter, length=245, thickness=5, rgba=[.4,.4,.4,1])
                # pandageom.plotAxisSelf(base.render, (cctpnt0+cctpnt1)/2+245*np.array([axx[0], axx[1], axx[2]]),
                #                 tmprtq85.getMat(), length=30, thickness=2)
                tmphand.setColor([0, 1, 0, .3])
                tmphand.reparentTo(base.render)
                self.handplotlist.append(tmphand)
            else:
                # for contact in result.getContacts():
                    # cp = contact.getManifoldPoint()
                    # pandageom.plotSphere(brchild, pos=cp.getLocalPointA(), radius=3, rgba=Vec4(1, 0, 0, 1))
                    # pandageom.plotSphere(brchild, pos=cp.getLocalPointB(), radius=3, rgba=Vec4(0, 0, 1, 1))
                tmphand.setColor([.5, 0, 0, .3])
                tmphand.reparentTo(base.render)
                self.handplotlist.append(tmphand)

    def plotObj(self):
        geomnodeobj = GeomNode('obj')
        geomnodeobj.addGeom(self.objgeom)
        npnodeobj = NodePath('obj')
        npnodeobj.attachNewNode(geomnodeobj)
        npnodeobj.reparentTo(base.render)

    def showAllGrips(self, handpkg):
        """
        showAllGrips

        :return:

        author: weiwei
        date: 20170206
        """

        for i in range(len(self.gripcontacts)):
            # if i < 15 and i > 2:
            #     for i in range(1,50,10):
            hndrotmat = self.griprotmats[i]
            hndjawwidth = self.gripjawwidth[i]
            # show grasps
            # tmphand = handpkg.newHandNM(hndcolor=[.2, .7, .2, .3])
            tmphand = handpkg.newHandNM(hndcolor=[1, 1, 1, .7])
            tmphand.setMat(pandanpmat4 = hndrotmat)
            tmphand.setJawwidth(hndjawwidth)
            tmphand.reparentTo(base.render)

class FreeAirGrip(object):
    """
    access data from db
    """

    def __init__(self, gdb, objname, handpkg):

        freeairgripdata = gdb.loadFreeAirGrip(objname, handname = handpkg.getHandName())
        if freeairgripdata is None:
            raise ValueError("Plan the freeairgrip first!")

        self.freegripids = freeairgripdata[0]
        self.freegripcontacts = freeairgripdata[1]
        self.freegripnormals = freeairgripdata[2]
        self.freegriprotmats = freeairgripdata[3]
        self.freegripjawwidth = freeairgripdata[4]

if __name__=='__main__':

    # def updateworld(world, task):
    #     world.doPhysics(globalClock.getDt())
    #     return task.cont

    base = pandactrl.World(camp=[700,300,700], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(this_dir, "objects", "sandpart.stl")
    # objpath = os.path.join(this_dir, "objects", "ttube.stl")
    # objpath = os.path.join(this_dir, "objects", "tool.stl")
    objpath = os.path.join(this_dir, "objects", "tool2.stl")
    # objpath = os.path.join(this_dir, "objects", "planewheel.stl")
    # objpath = os.path.join(this_dir, "objects", "planelowerbody.stl")
    # objpath = os.path.join(this_dir, "objects", "planefrontstay.stl")
    # objpath = os.path.join(this_dir, "objects", "planerearstay.stl")

    handpkg = hrp5threenm
    freegriptst = Freethreegrip(objpath, handpkg, readser=False, torqueresist = 100)

    freegriptst.segShow(base, togglesamples=False, togglenormals=False,
                        togglesamples_ref=False, togglenormals_ref=False,
                        togglesamples_refcls=False, togglenormals_refcls=False, alpha =1)

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
    #     freegriptst = Freegrip(objpath, ser=False, torqueresist = 50)
    #     freegriptst.removeFgrpcc(base)
    #     freegriptst.removeHndcc(base)
    #     toc = time.clock()
    #     print toc-tic
    #     fo.write(os.path.basename(objpath)+' '+str(toc-tic)+'\n')
    # fo.close()

    # geom = None
    # for i, faces in enumerate(freegriptst.objtrimesh.facets()):
    #     rgba = [np.random.random(),np.random.random(),np.random.random(),1]
    #     # geom = pandageom.packpandageom(freegriptst.objtrimesh.vertices, freegriptst.objtrimesh.face_normals[faces], freegriptst.objtrimesh.faces[faces])
    #     # compute facet normal
    #     facetnormal = np.sum(freegriptst.objtrimesh.face_normals[faces], axis=0)
    #     facetnormal = facetnormal/np.linalg.norm(facetnormal)
    #     geom = pandageom.packpandageom(freegriptst.objtrimesh.vertices +
    #                             np.tile(0 * facetnormal,
    #                                     [freegriptst.objtrimesh.vertices.shape[0], 1]),
    #                             freegriptst.objtrimesh.face_normals[faces],
    #                             freegriptst.objtrimesh.faces[faces])
    #     node = GeomNode('piece')
    #     node.addGeom(geom)
    #     star = NodePath('piece')
    #     star.attachNewNode(node)
    #     star.setColor(Vec4(rgba[0],rgba[1],rgba[2],rgba[3]))
    #     # star.setColor(Vec4(.7,.4,0,1))
    #     star.setTwoSided(True)
    #     star.reparentTo(base.render)

    # freegriptst.removeFgrpcc(base)
    # def updateshow(task):
    #     freegriptst.pairShow(base, togglecontacts=True, togglecontactnormals=True)
    #     # print task.delayTime
    #     # if abs(task.delayTime-13) < 1:
    #     #     task.delayTime -= 12.85
    #     return task.again
    #
    # taskMgr.doMethodLater(.5, updateshow, "tickTask")

    freegriptst.removeFgrpcc(base)
    freegriptst.removeHndcc(base)

    gdb = db.GraspDB()
    freegriptst.saveToDB(gdb)

    # def updateshow(task):
    #     # freegriptst.removeFgrpccShow(base)
    #     # freegriptst.removeFgrpccShowLeft(base)
    #     freegriptst.removeHndccShow(base)
    #     # print task.delayTime
    #     # if abs(task.delayTime-13) < 1:
    #     #     task.delayTime -= 12.85
    #     return task.again
    # #
    # taskMgr.doMethodLater(3, updateshow, "tickTask")
    # freegriptst.removeFgrpcc(base)
    # freegriptst.removeHndcc(base)

    # def updateworld(world, task):
    #     world.doPhysics(globalClock.getDt())
    #     return task.cont
    #
    # base.taskMgr.add(updateworld, "updateworld", extraArgs=[freegriptst.bulletworld], appendTask=True)
    #
    # debugNode = BulletDebugNode('Debug')
    # debugNode.showWireframe(True)
    # debugNode.showConstraints(True)
    # debugNode.showBoundingBoxes(False)
    # debugNode.showNormals(False)
    # bullcldrnp = base.render.attachNewNode("bulletcollider")
    # debugNP = bullcldrnp.attachNewNode(debugNode)
    # debugNP.show()
    # freegriptst.bulletworld.setDebugNode(debugNP.node())
    # taskMgr.add(updateworld, "updateworld", extraArgs=[freegriptst.bulletworld], appendTask=True)

    freegriptst.showAllGrips(handpkg)
    base.run()