#!/usr/bin/python

import itertools
import os

import numpy as np
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletTriangleMesh
from panda3d.bullet import BulletTriangleMeshShape
from panda3d.bullet import BulletWorld
from panda3d.core import *
from shapely.geometry import Point
from shapely.geometry import Polygon
from shapely.geometry import MultiPolygon
from sklearn.cluster import KMeans
from sklearn.neighbors import RadiusNeighborsClassifier

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pg
import trimesh
from utils import robotmath

from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
from panda3d.ode import *
import random
import time

class Gripper(object):

    def __init__(self, base, odespace, odeworld, fgrlen = 50.0, handwidth = 20.0):
        self.base = base
        self.pggen = pg.PandaGeomGen()
        self.handnp = NodePath("hand")

        self.handlen = fgrlen
        self.handwidth = handwidth
        self.odespace = odespace
        self.odeworld = odeworld

        # palm, no geom
        self.palmbody = OdeBody(self.odeworld)
        palmmass = OdeMass()
        palmmass.setSphere(.2,1)
        self.palmbody.setMass(palmmass)
        self.palmgeom = self.pggen.genBox(10.0, 10.0, 2.0)
        self.palmgeom.reparentTo(self.base.render)
        # linear joint between palm and environment
        self.palmenvjnt = OdeSliderJoint(self.odeworld)
        self.palmenvjnt.attachBody(self.palmbody, 0)
        self.palmenvjnt.setAxis(0,0,-1)
        self.palmenvjnt.setParamVel(0)
        self.palmenvjnt.setParamFMax(10000000)

        # fgr0
        self.fgr0body = OdeBody(self.odeworld)
        fgr0mass = OdeMass()
        fgr0mass.setSphere(.1,1)
        self.fgr0body.setMass(palmmass)
        self.fgr0geom = self.pggen.genBox(10.0, 2.0, self.handlen)
        self.fgr0geom.reparentTo(self.base.render)
        fgr0trimesh = OdeTriMeshData(self.fgr0geom, True)
        fgr0geom = OdeTriMeshGeom(self.odespace, fgr0trimesh)
        fgr0geom.setBody(self.fgr0body)
        # linear joint between fgr0 and palm
        self.fgr0palmjnt = OdeSliderJoint(self.odeworld)
        self.fgr0palmjnt.attach(self.fgr0body, self.palmbody)
        self.fgr0palmjnt.setAxis(0,1,0)
        self.fgr0palmjnt.setParamVel(0)
        self.fgr0palmjnt.setParamFMax(10000000)

        # fgr1
        self.fgr1body = OdeBody(self.odeworld)
        fgr1mass = OdeMass()
        fgr1mass.setSphere(.1,1)
        self.fgr1body.setMass(palmmass)
        self.fgr1geom = self.pggen.genBox(10.0, 2.0, self.handlen)
        self.fgr1geom.reparentTo(self.base.render)
        fgr1trimesh = OdeTriMeshData(self.fgr1geom, True)
        fgr1geom = OdeTriMeshGeom(self.odespace, fgr1trimesh)
        fgr1geom.setBody(self.fgr1body)
        # linear joint between fgr1 and palm
        self.fgr1palmjnt = OdeSliderJoint(self.odeworld)
        self.fgr1palmjnt.attach(self.fgr1body, self.palmbody)
        self.fgr1palmjnt.setAxis(0,-1,0)
        self.fgr1palmjnt.setParamVel(0)
        self.fgr1palmjnt.setParamFMax(10000000)

        self.bopenhand = False
        self.bclosehand = False

        self.palmbody.setPosition(0,0,1000.0)
        self.fgr0body.setPosition(0,0,1000.0)
        self.fgr1body.setPosition(0,0,1000.0)
        self.update()

        self.resetgrip()

    def resethand(self):

        self.palmbody.setPosition(0,0,100.0)
        self.fgr0body.setPosition(0,0,100.0)
        self.fgr1body.setPosition(0,0,100.0)

        self.resetgrip()

    def resetgrip(self):
        self.bopen = True
        self.bdown = False
        self.bgrip = False
        self.bup = False

    # def resetopen(self):
    #     self.bopenhand = True
    #
    # def openHandProc(self):
    #     if self.bopenhand:
    #         self.fgr0palmjnt.setParamVel(10)
    #         self.fgr1palmjnt.setParamVel(10)
    #         fgr0bodypos = self.fgr0body.getPosition()
    #         fgr1bodypos = self.fgr1body.getPosition()
    #         vec01 = fgr1bodypos-fgr0bodypos
    #         vec01len = vec01.length()
    #         if vec01len > self.handwidth:
    #             self.fgr0palmjnt.setParamVel(0)
    #             self.fgr1palmjnt.setParamVel(0)
    #             self.bopenhand = False
    #
    # def closeHand(self):
    #     self.bclosehand = True
    #
    # def closeHandProc(self):
    #     self.bopenhand = True
    #     self.fgr0palmjnt.setParamVel(-10)
    #     self.fgr1palmjnt.setParamVel(-10)
    #     # force control
    #     # fgr0feedback = self.fgr0palmjnt.getFeedback()
    #     # fgr0force = 0
    #     # if fgr0feedback is not None:
    #     #     fgr0force = fgr0feedback.getForce1()
    #     # fgr1feedback = self.fgr1palmjnt.getFeedback()
    #     # fgr1force = 0
    #     # if fgr1feedback is not None:
    #     #     fgr1force = fgr1feedback.getForce1()
    #     # if fgr0force > 15 and fgr1force > 15:
    #     #     self.fgr0palmjnt.setParamVel(0)
    #     #     self.fgr1palmjnt.setParamVel(0)
    #     #     self.bclosehand = False
    #     fgr0bodypos = self.fgr0body.getPosition()
    #     fgr1bodypos = self.fgr1body.getPosition()
    #     vec01 = fgr1bodypos-fgr0bodypos
    #     vec01len = vec01.length()
    #     if vec01len < 5.0:
    #         self.fgr0palmjnt.setParamVel(0)
    #         self.fgr1palmjnt.setParamVel(0)
    #         self.bopenhand = False

    def gripProc(self):
        if self.bopen:
            self.fgr0palmjnt.setParamVel(5)
            self.fgr1palmjnt.setParamVel(5)
            fgr0bodypos = self.fgr0body.getPosition()
            fgr1bodypos = self.fgr1body.getPosition()
            vec01 = fgr1bodypos-fgr0bodypos
            vec01len = vec01.length()
            if vec01len > self.handwidth:
                self.fgr0palmjnt.setParamVel(0)
                self.fgr1palmjnt.setParamVel(0)
                self.bopen = False
                self.bdown = True
                self.bgrip = False
                self.bup = False
        if self.bdown:
            self.palmenvjnt.setParamVel(10)
            if self.getTipPos() < 5.0:
                self.palmenvjnt.setParamVel(0)
                self.bopen = False
                self.bdown = False
                self.bgrip = True
                self.bup = False
        if self.bgrip:
            # closehand
            self.fgr0palmjnt.setParamVel(-5)
            self.fgr1palmjnt.setParamVel(-5)
            fgr0bodypos = self.fgr0body.getPosition()
            fgr1bodypos = self.fgr1body.getPosition()
            vec01 = fgr1bodypos-fgr0bodypos
            vec01len = vec01.length()
            if vec01len < 5.0:
                self.fgr0palmjnt.setParamVel(0)
                self.fgr1palmjnt.setParamVel(0)
                self.bopen = False
                self.bdown = False
                self.bgrip = False
                self.bup = True
        if self.bup:
            # move up
            self.palmenvjnt.setParamVel(-5)
            if self.getTipPos() > 100.0:
                self.palmenvjnt.setParamVel(0)
                self.bopen = False
                self.bdown = False
                self.bgrip = False
                self.bup = False
                return True
        return False

    def getTipPos(self):
        self.fgr0bodypos = self.fgr0body.getPosition()
        return self.fgr0bodypos[2]-self.handlen/2.0

    def update(self):
        # palm
        pospalm = self.palmbody.getPosition()
        rotpalm = self.palmbody.getRotation()
        matpalm = Mat4(rotpalm)
        matpalm.setRow(3, pospalm)
        self.palmgeom.setMat(matpalm)
        # fgr0
        posfgr0 = self.fgr0body.getPosition()
        rotfgr0= self.fgr0body.getRotation()
        matfgr0 = Mat4(rotfgr0)
        matfgr0.setRow(3, posfgr0)
        self.fgr0geom.setMat(matfgr0)
        # fgr1
        posfgr1 = self.fgr1body.getPosition()
        rotfgr1= self.fgr1body.getRotation()
        matfgr1 = Mat4(rotfgr1)
        matfgr1.setRow(3, posfgr1)
        self.fgr1geom.setMat(matfgr1)

if __name__=='__main__':

    base = pandactrl.World(camp=[0,0,550], lookatp=[0,0,0])
    pg.plotAxisSelf(base.render, length = 10, thickness = 1)

    odeWorld = OdeWorld()
    odeWorld.setGravity(0, 0, -9.81)

    odeSpace = OdeSimpleSpace()

    gper = Gripper(base, odeSpace, odeWorld)
    gper.resethand()

    def updateshow(task):
        odeWorld.quickStep(globalClock.getDt())
        gper.update()
        gper.gripProc()

        # if gper.getTipWidth() > 0.0:
        #     if gper.getTipDepth() > 10.0:
        #         gper.downHandStep(step=.5)
        #         # gper.update()
        #         pass
        #     else:
        #         gper.closeHandStep()
        # else:
        #     gper.dettach()
        #     gper.reset()

        return task.again
    taskMgr.add(updateshow, "updateshow")

    # dcam = loader.loadShader("depthmap.sha")
    # # render everything through this camera and shader
    # base.render.setShader(dcam)
    # # loadPrcFileData('', 'show-buffers 1')

    base.run()
