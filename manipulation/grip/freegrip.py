#!/usr/bin/python

import os

import MySQLdb as mdb
import numpy as np
from manipulation.grip.robotiq85 import rtq85nm
from panda3d.bullet import BulletWorld
from panda3d.core import *

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pandageom
from manipulation.grip import freegripcontactpairs as fgcp
from utils import collisiondetection as cd
from utils import dbcvt as dc
from utils import robotmath as rm
from database import dbaccess as db


class Freegrip(fgcp.FreegripContactpairs):

    def __init__(self, objpath):
        """
        initialization

        :param objpath: path of the object

        author: weiwei
        date: 20161201, osaka
        """

        super(self.__class__, self).__init__(objpath)
        self.removeBadSamples()
        self.clusterFacetSamplesRNN(reduceRadius=10)
        self.planContactpairs()
        self.loadRtq85Models()
        # gripcontactpairs_precc is the gripcontactpairs ([[p0,p1,p2],[p0',p1',p2']] pairs) after precc (collision free)
        # gripcontactpairnormals_precc is the gripcontactpairnormals ([[n0,n1,n2],[n0',n1',n2']] pairs) after precc
        # likewise, gripcontactpairfacets_precc is the [faceid0, faceid1] pair corresponding to the upper two
        self.gripcontactpairs_precc = None
        self.gripcontactpairnormals_precc = None
        self.gripcontactpairfacets_precc = None

        # the final results: gripcontacts: a list of [cct0, cct1]
        # griprotmats: a list of Mat4
        # gripcontactnormals: a list of [nrml0, nrml1]
        self.gripcontacts = None
        self.griprotmats = None
        self.gripjawwidth = None
        self.gripcontactnormals = None

        self.bulletworld = BulletWorld()
        # prepare the model for collision detection
        geom = pandageom.packpandageom(self.objtrimesh.vertices, self.objtrimesh.face_normals, self.objtrimesh.faces)
        self.objmeshbullnode = cd.genCollisionMeshGeom(geom)
        self.bulletworld.attachRigidBody(self.objmeshbullnode)

        # for plot
        self.rtq85plotlist = []

        # for dbupdate
        self.dbobjname = os.path.splitext(os.path.basename(objpath))[0]

    def loadRtq85Models(self):
        """
        load the rtq85 model and its fgrprecc model
        :return:
        """
        self.rtq85hnd = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, .1])
        rtq85pccpath = Filename.fromOsSpecific(os.path.join(this_dir, "robotiq85/rtq85egg", "robotiq_85_tip_precc.egg"))
        self.rtq85fgrpcc_uninstanced = loader.loadModel(rtq85pccpath)

    def removeHndcc(self, base, discretesize=8):
        """
        Handcc means hand collision detection

        :param discretesize: the number of hand orientations
        :return:

        author: weiwei
        date: 20161212, tsukuba
        """

        # isplotted = 0

        # if self.rtq85plotlist:
        #     for rtq85plotnode in self.rtq85plotlist:
        #         rtq85plotnode.removeNode()
        # self.rtq85plotlist = []

        self.gripcontacts = []
        self.griprotmats = []
        self.gripjawwidth = []
        self.gripcontactnormals = []

        plotoffsetfp = 2

        self.counter = 0

        while self.counter < self.facetpairs.shape[0]:
            print str(self.counter) + "/" + str(self.facetpairs.shape[0]-1)

            facetpair = self.facetpairs[self.counter]
            facetidx0 = facetpair[0]
            facetidx1 = facetpair[1]

            for j, contactpair in enumerate(self.gripcontactpairs_precc[self.counter]):
                for angleid in range(discretesize):
                    cctpnt0 = contactpair[0] + plotoffsetfp * self.facetnormals[facetidx0]
                    cctpnt1 = contactpair[1] + plotoffsetfp * self.facetnormals[facetidx1]
                    cctnormal0 = self.gripcontactpairnormals_precc[self.counter][j][0]
                    cctnormal1 = [-cctnormal0[0], -cctnormal0[1], -cctnormal0[2]]
                    tmprtq85 = self.rtq85hnd
                    # tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, .1])
                    # save initial hand pose
                    initmat = tmprtq85.getMat()
                    fgrdist = np.linalg.norm((cctpnt0 - cctpnt1))
                    tmprtq85.setJawwidth(fgrdist)
                    tmprtq85.lookAt(cctnormal0[0], cctnormal0[1], cctnormal0[2])
                    rotax = [0, 1, 0]
                    rotangle = 360.0 / discretesize * angleid
                    rotmat = rm.rodrigues(rotax, rotangle)
                    tmprtq85.setMat(pandageom.cvtMat4(rotmat) * tmprtq85.getMat())
                    axx = tmprtq85.getMat().getRow3(0)
                    # 130 is the distance from hndbase to fingertip
                    cctcenter = (cctpnt0 + cctpnt1) / 2 + 145 * np.array([axx[0], axx[1], axx[2]])
                    tmprtq85.setPos(Point3(cctcenter[0], cctcenter[1], cctcenter[2]))

                    # collision detection
                    hndbullnode = cd.genCollisionMeshMultiNp(tmprtq85.rtq85np, base.render)
                    self.bulletworld.attachRigidBody(hndbullnode)
                    result = self.bulletworld.contactTest(self.objmeshbullnode)

                    if not result.getNumContacts():
                        self.gripcontacts.append(contactpair)
                        self.griprotmats.append(tmprtq85.getMat())
                        self.gripjawwidth.append(fgrdist)
                        self.gripcontactnormals.append(self.gripcontactpairnormals_precc[self.counter][j])
                        # pg.plotDumbbell(base.render, (cctpnt0+cctpnt1)/2, cctcenter, length=245, thickness=5, rgba=[.4,.4,.4,1])
                        # pg.plotAxisSelf(base.render, (cctpnt0+cctpnt1)/2+245*np.array([axx[0], axx[1], axx[2]]),
                        #                 tmprtq85.getMat(), length=30, thickness=2)
                        # tmprtq85.setColor([.5, .5, .5, 1])
                        # tmprtq85.reparentTo(base.render)
                        # self.rtq85plotlist.append(tmprtq85)
                        # isplotted = 1

                    self.bulletworld.removeRigidBody(hndbullnode)
                    # reset initial hand pose
                    tmprtq85.setMat(initmat)
            self.counter+=1
        self.counter = 0

    def removeFgrpcc(self, base):
        """
        Fgrpcc means finger pre collision detection

        :return:

        author: weiwei
        date: 20161212, tsukuba
        """

        self.gripcontactpairs_precc = []
        self.gripcontactpairnormals_precc = []
        self.gripcontactpairfacets_precc = []

        plotoffsetfp = 2

        self.counter = 0

        while self.counter < self.facetpairs.shape[0]:
            print str(self.counter) + "/" + str(self.facetpairs.shape[0]-1)
            self.gripcontactpairs_precc.append([])
            self.gripcontactpairnormals_precc.append([])
            self.gripcontactpairfacets_precc.append([])

            facetpair = self.facetpairs[self.counter]
            facetidx0 = facetpair[0]
            facetidx1 = facetpair[1]

            for j, contactpair in enumerate(self.gripcontactpairs[self.counter]):
                cctpnt0 = contactpair[0] + plotoffsetfp * self.facetnormals[facetidx0]
                cctpnt1 = contactpair[1] + plotoffsetfp * self.facetnormals[facetidx1]
                cctnormal0 = self.facetnormals[facetidx0]
                cctnormal1 = [-cctnormal0[0], -cctnormal0[1], -cctnormal0[2]]
                rtq85pcc0 = NodePath("rtq85fgrpcc0")
                self.rtq85fgrpcc_uninstanced.instanceTo(rtq85pcc0)
                rtq85pcc0.setPos(cctpnt0[0], cctpnt0[1], cctpnt0[2])
                rtq85pcc0.lookAt(cctpnt0[0] + cctnormal0[0], cctpnt0[1] + cctnormal0[1],
                                 cctpnt0[2] + cctnormal0[2])
                rtq85pcc1 = NodePath("rtq85fgrpcc1")
                self.rtq85fgrpcc_uninstanced.instanceTo(rtq85pcc1)
                rtq85pcc1.setPos(cctpnt1[0], cctpnt1[1], cctpnt1[2])
                rtq85pcc1.lookAt(cctpnt1[0] + cctnormal1[0], cctpnt1[1] + cctnormal1[1],
                                 cctpnt1[2] + cctnormal1[2])


                # prepare the model for collision detection
                facetmesh0bullnode = cd.genCollisionMeshNp(rtq85pcc0, base.render)
                self.bulletworld.attachRigidBody(facetmesh0bullnode)
                facetmesh1bullnode = cd.genCollisionMeshNp(rtq85pcc1, base.render)
                self.bulletworld.attachRigidBody(facetmesh1bullnode)
                result = self.bulletworld.contactTest(self.objmeshbullnode)
                self.bulletworld.removeRigidBody(facetmesh0bullnode)
                self.bulletworld.removeRigidBody(facetmesh1bullnode)

                if not result.getNumContacts():
                    self.gripcontactpairs_precc[-1].append(contactpair)
                    self.gripcontactpairnormals_precc[-1].append(self.gripcontactpairnormals[self.counter][j])
                    self.gripcontactpairfacets_precc[-1].append(self.gripcontactpairfacets[self.counter])
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

        sql = "SELECT * FROM freeairgrip, object WHERE freeairgrip.idobject = object.idobject AND \
                object.objname LIKE '%s'" % self.dbobjname
        result = gdb.execute(sql)
        if not result:
            sql = "SELECT idobject FROM object WHERE objname LIKE '%s'" % self.dbobjname
            returnlist = gdb.execute(sql)[0]
            if len(returnlist) != 0:
                idobject = returnlist[0]
            else:
                sql = "INSERT INTO object(objname) VALUES('%s')" % self.dbobjname
                idobject = gdb.execute(sql)
            for i in range(len(self.gripcontacts)):
                sql = "INSERT INTO freeairgrip(idobject, contactpnt0, contactpnt1, \
                        contactnormal0, contactnormal1, rotmat, jawwidth) \
                       VALUES('%s', '%s', '%s', '%s', '%s', '%s', '%s')" % \
                      (idobject, dc.v3ToStr(self.gripcontacts[i][0]), dc.v3ToStr(self.gripcontacts[i][1]),
                       dc.v3ToStr(self.gripcontactnormals[i][0]), dc.v3ToStr(self.gripcontactnormals[i][1]),
                       dc.mat4ToStr(self.griprotmats[i]), str(self.gripjawwidth[i]))
                gdb.execute(sql)
        else:
            print "Grasps already saved or duplicated filename!"

    def removeFgrpccShow(self, base):
        """
        Fgrpcc means finger pre collision detection
        This one is specially written for demonstration

        :return:

        author: weiwei
        date: 20161201, osaka
        """

        plotoffsetfp = 2

        # np0 = base.render.find("**/pair0")
        # if np0:
        #     np0.removeNode()
        # np1 = base.render.find("**/pair1")
        # if np1:
        #     np1.removeNode()
        #
        # np0collection = base.render.findAllMatches("**/rtq85fgrpcc0")
        # for np0 in np0collection:
        #     np0.removeNode()
        # np1collection = base.render.findAllMatches("**/rtq85fgrpcc1")
        # for np1 in np1collection:
        #     np1.removeNode()
        #
        # npscollection = base.render.findAllMatches("**/sphere")
        # for nps in npscollection:
        #     nps.removeNode()

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
        geomfacet0 = pandageom.packpandageom(self.objtrimesh.vertices+
                                       np.tile(plotoffsetfp*self.facetnormals[facetidx0],
                                               [self.objtrimesh.vertices.shape[0],1]),
                                       self.objtrimesh.face_normals[self.facets[facetidx0]],
                                       self.objtrimesh.faces[self.facets[facetidx0]])
        geomfacet1 = pandageom.packpandageom(self.objtrimesh.vertices+
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
        for j, contactpair in enumerate(self.gripcontactpairs[self.counter]):
            cctpnt0 = contactpair[0] + plotoffsetfp * self.facetnormals[facetidx0]
            cctpnt1 = contactpair[1] + plotoffsetfp * self.facetnormals[facetidx1]
            cctnormal0 = self.gripcontactpairnormals[self.counter][j][0]
            cctnormal1 = [-cctnormal0[0], -cctnormal0[1], -cctnormal0[2]]
            rtq85pcc0 = NodePath("rtq85fgrpcc0")
            self.rtq85fgrpcc_uninstanced.instanceTo(rtq85pcc0)
            rtq85pcc0.setPos(cctpnt0[0], cctpnt0[1], cctpnt0[2])
            rtq85pcc0.lookAt(cctpnt0[0] + cctnormal0[0], cctpnt0[1] + cctnormal0[1], cctpnt0[2] + cctnormal0[2])
            rtq85pcc1 = NodePath("rtq85fgrpcc1")
            self.rtq85fgrpcc_uninstanced.instanceTo(rtq85pcc1)
            rtq85pcc1.setPos(cctpnt1[0], cctpnt1[1], cctpnt1[2])
            rtq85pcc1.lookAt(cctpnt1[0] + cctnormal1[0], cctpnt1[1] + cctnormal1[1], cctpnt1[2] + cctnormal1[2])

            # prepare the model for collision detection
            facetmesh0bullnode = cd.genCollisionMeshNp(rtq85pcc0, brchild)
            self.bulletworld.attachRigidBody(facetmesh0bullnode)
            facetmesh1bullnode = cd.genCollisionMeshNp(rtq85pcc1, brchild)
            self.bulletworld.attachRigidBody(facetmesh1bullnode)

            result = self.bulletworld.contactTest(self.objmeshbullnode)
            self.bulletworld.removeRigidBody(facetmesh0bullnode)
            self.bulletworld.removeRigidBody(facetmesh1bullnode)

            for contact in result.getContacts():
                cp = contact.getManifoldPoint()
                pandageom.plotSphere(brchild, pos=cp.getLocalPointA(), radius=3, rgba=Vec4(1, 0, 0, 1))

            if result.getNumContacts():
                rtq85pcc0.setColor(1, 0, 0, .3)
                rtq85pcc1.setColor(1, 0, 0, .3)
            else:
                rtq85pcc0.setColor(1, 1, 1, .3)
                rtq85pcc1.setColor(1, 1, 1, .3)

            rtq85pcc0.setTransparency(TransparencyAttrib.MAlpha)
            rtq85pcc1.setTransparency(TransparencyAttrib.MAlpha)
            rtq85pcc0.reparentTo(brchild)
            rtq85pcc1.reparentTo(brchild)
            pandageom.plotArrow(star0, spos=cctpnt0,
                            epos=cctpnt0 + plotoffsetfp*self.facetnormals[facetidx0] + cctnormal0,
                            rgba=[facetcolorarray[facetidx0][0], facetcolorarray[facetidx0][1],
                                  facetcolorarray[facetidx0][2], facetcolorarray[facetidx0][3]], length=10)
            pandageom.plotArrow(star1, spos=cctpnt1,
                            epos=cctpnt1 + plotoffsetfp*self.facetnormals[facetidx1] + cctnormal1,
                            rgba=[facetcolorarray[facetidx1][0], facetcolorarray[facetidx1][1],
                                  facetcolorarray[facetidx1][2], facetcolorarray[facetidx1][3]], length=10)

    def removeFgrpccShowLeft(self, base):
        """
        Fgrpcc means finger pre collision detection
        This one is specially written for demonstration
        Plot the available grips

        :return:

        author: weiwei
        date: 20161212, tsukuba
        """

        plotoffsetfp = 2

        self.counter += 1
        if self.counter >= self.facetpairs.shape[0]:
            return
        else:
            print str(self.counter) + "/" + str(self.facetpairs.shape[0]-1)

            facetpair = self.facetpairs[self.counter]
            facetidx0 = facetpair[0]
            facetidx1 = facetpair[1]

            for j, contactpair in enumerate(self.gripcontactpairs[self.counter]):
                cctpnt0 = contactpair[0] + plotoffsetfp * self.facetnormals[facetidx0]
                cctpnt1 = contactpair[1] + plotoffsetfp * self.facetnormals[facetidx1]
                cctnormal0 = self.facetnormals[facetidx0]
                cctnormal1 = [-cctnormal0[0], -cctnormal0[1], -cctnormal0[2]]
                rtq85pcc0 = NodePath("rtq85fgrpcc0")
                self.rtq85fgrpcc_uninstanced.instanceTo(rtq85pcc0)
                rtq85pcc0.setPos(cctpnt0[0], cctpnt0[1], cctpnt0[2])
                rtq85pcc0.lookAt(cctpnt0[0] + cctnormal0[0], cctpnt0[1] + cctnormal0[1], cctpnt0[2] + cctnormal0[2])
                rtq85pcc1 = NodePath("rtq85fgrpcc1")
                self.rtq85fgrpcc_uninstanced.instanceTo(rtq85pcc1)
                rtq85pcc1.setPos(cctpnt1[0], cctpnt1[1], cctpnt1[2])
                rtq85pcc1.lookAt(cctpnt1[0] + cctnormal1[0], cctpnt1[1] + cctnormal1[1], cctpnt1[2] + cctnormal1[2])

                # prepare the model for collision detection
                facetmesh0bullnode = cd.genCollisionMeshNp(rtq85pcc0, base.render)
                self.bulletworld.attachRigidBody(facetmesh0bullnode)
                facetmesh1bullnode = cd.genCollisionMeshNp(rtq85pcc1, base.render)
                self.bulletworld.attachRigidBody(facetmesh1bullnode)
                result = self.bulletworld.contactTest(self.objmeshbullnode)
                self.bulletworld.removeRigidBody(facetmesh0bullnode)
                self.bulletworld.removeRigidBody(facetmesh1bullnode)

                if not result.getNumContacts():
                    rtq85pcc0.setColor(1, 1, 1, .3)
                    rtq85pcc1.setColor(1, 1, 1, .3)
                    rtq85pcc0.setTransparency(TransparencyAttrib.MAlpha)
                    rtq85pcc1.setTransparency(TransparencyAttrib.MAlpha)
                    rtq85pcc0.reparentTo(base.render)
                    rtq85pcc1.reparentTo(base.render)

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

        if self.counter is 0:
            self.gripcontacts = []
            self.gripnormals = []
            self.griprotmats = []
            self.gripjawwidth = []
            # access to db
            db = mdb.connect("localhost", "weiweilab", "weiweilab", "freegrip")
            cursor = db.cursor()
            sql = "select contactpnt0, contactpnt1, contactnormal0, contactnormal1, rotmat, jawwidth from freeair"
            try:
                cursor.execute(sql)
            except mdb.Error as e:
                print "MySQL Error [%d]: %s" % (e.args[0], e.args[1])
                db.rollback()
                raise mdb.Error
            data = list(cursor.fetchall())
            for i in range(len(data)):
                self.gripcontacts.append([dc.strToV3(data[i][0]), dc.strToV3(data[i][1])])
                self.gripnormals.append([dc.strToV3(data[i][2]), dc.strToV3(data[i][3])])
                self.griprotmats.append(dc.strToMat4(data[i][4]))
                self.gripjawwidth.append(float(data[i][5]))
            db.close()

        if self.rtq85plotlist:
            for rtq85plotnode in self.rtq85plotlist:
                rtq85plotnode.removeNode()
        self.rtq85plotlist = []

        if self.counter >= len(self.gripcontacts):
            return
        else:
            print str(self.counter) + "/" + str(len(self.gripcontacts)-1)
            rotmat = self.griprotmats[self.counter]
            tmprtq85 = rtq85nm.Rtq85NM(hndcolor=[1, 0, 0, .1])
            tmprtq85.setMat(rotmat)
            tmprtq85.setJawwidth(self.gripjawwidth[self.counter])

            tmprtq85.setColor([.5, .5, .5, 1])
            tmprtq85.reparentTo(base.render)
            self.rtq85plotlist.append(tmprtq85)
            self.counter += 1


if __name__=='__main__':


    # def updateworld(world, task):
    #     world.doPhysics(globalClock.getDt())
    #     return task.cont

    base = pandactrl.World(camp=[700,300,700], lookatp=[0,0,0])
    this_dir, this_filename = os.path.split(__file__)
    objpath = os.path.join(this_dir, "objects", "ttube.stl")
    freegriptst = Freegrip(objpath)

    # freegriptst.segShow(base, togglesamples=False, togglenormals=False,
    #                     togglesamples_ref=False, togglenormals_ref=False,
    #                     togglesamples_refcls=False, togglenormals_refcls=False)

    freegriptst.removeFgrpcc(base)
    freegriptst.removeHndcc(base)

    gdb = db.GraspDB()
    freegriptst.saveToDB(gdb)

    # def updateshow(task):
    # #     freegriptst.removeFgrpccShow(base)
    # #     freegriptst.removeFgrpccShowLeft(base)
    #     freegriptst.removeHndccShow(base)
    # #     # print task.delayTime
    # #     # if abs(task.delayTime-13) < 1:
    # #     #     task.delayTime -= 12.85
    #     return task.again
    #
    # taskMgr.doMethodLater(.3, updateshow, "tickTask")
    # # freegriptst.removeFgrpcc(base)
    # # freegriptst.removeHndcc(base)

    # taskMgr.add(updateworld, "updateworld", extraArgs=[freegriptst.bulletworld], appendTask=True)
    base.run()