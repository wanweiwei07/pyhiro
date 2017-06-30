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
import gripper

class ODESim(object):

    def __init__(self, base, objpath, nobj = 5):
        self.base = base
        self.nobj = nobj

        self.smiley = loader.loadModel(objpath)
        self.smileyCount = 0

        self.setupODE()
        self.addGround()

        self.partlist = []
        self.gripper = gripper.Gripper(base = self.base, odespace = self.odespace, odeworld = self.odeworld)


        # test
        # self.pggen = pg.PandaGeomGen()
        # self.handlen = 50.0
        #
        # # palm, no geom
        # self.palmbody = OdeBody(self.odeworld)
        # palmmass = OdeMass()
        # palmmass.setSphere(.2,1)
        # self.palmbody.setMass(palmmass)
        # self.palmgeom = self.pggen.genBox(10.0, 10.0, 2.0)
        # self.palmgeom.reparentTo(self.base.render)
        # # linear joint between palm and environment
        # self.palmenvjnt = OdeSliderJoint(self.odeworld)
        # self.palmenvjnt.attachBody(self.palmbody, 0)
        # self.palmenvjnt.setAxis(0,0,-1)
        # self.palmenvjnt.setParamVel(0)
        # self.palmenvjnt.setParamFMax(1000000)
        #
        # # fgr0
        # self.fgr0body = OdeBody(self.odeworld)
        # fgr0mass = OdeMass()
        # fgr0mass.setSphere(.1,1)
        # self.fgr0body.setMass(palmmass)
        # self.fgr0geom = self.pggen.genBox(10.0, 2.0, self.handlen)
        # self.fgr0geom.reparentTo(self.base.render)
        # fgr0trimesh = OdeTriMeshData(self.fgr0geom, True)
        # fgr0geom = OdeTriMeshGeom(self.odespace, fgr0trimesh)
        # fgr0geom.setBody(self.fgr0body)
        # # linear joint between fgr0 and palm
        # self.fgr0palmjnt = OdeSliderJoint(self.odeworld)
        # self.fgr0palmjnt.attach(self.fgr0body, self.palmbody)
        # self.fgr0palmjnt.setAxis(0,1,0)
        # self.fgr0palmjnt.setParamVel(0)
        # self.fgr0palmjnt.setParamFMax(1000000)
        #
        # # fgr1
        # self.fgr1body = OdeBody(self.odeworld)
        # fgr1mass = OdeMass()
        # fgr1mass.setSphere(.1,1)
        # self.fgr1body.setMass(palmmass)
        # self.fgr1geom = self.pggen.genBox(10.0, 2.0, self.handlen)
        # self.fgr1geom.reparentTo(self.base.render)
        # fgr1trimesh = OdeTriMeshData(self.fgr1geom, True)
        # fgr1geom = OdeTriMeshGeom(self.odespace, fgr1trimesh)
        # fgr1geom.setBody(self.fgr1body)
        # # linear joint between fgr1 and palm
        # self.fgr1palmjnt = OdeSliderJoint(self.odeworld)
        # self.fgr1palmjnt.attach(self.fgr1body, self.palmbody)
        # self.fgr1palmjnt.setAxis(0,-1,0)
        # self.fgr1palmjnt.setParamVel(0)
        # self.fgr1palmjnt.setParamFMax(1000000)
        #
        # self.bopenhand = False
        # self.bclosehand = False
        #
        # self.palmbody.setPosition(0,0,20.0)
        # self.fgr0body.setPosition(0,0,20.0)
        # self.fgr1body.setPosition(0,0,20.0)
        #
        # # palm
        # pospalm = self.palmbody.getPosition()
        # rotpalm = self.palmbody.getRotation()
        # matpalm = Mat4(rotpalm)
        # matpalm.setRow(3, pospalm)
        # self.palmgeom.setMat(matpalm)
        # # fgr0
        # posfgr0 = self.fgr0body.getPosition()
        # rotfgr0= self.fgr0body.getRotation()
        # matfgr0 = Mat4(rotfgr0)
        # matfgr0.setRow(3, posfgr0)
        # self.fgr0geom.setMat(matfgr0)
        # # fgr1
        # posfgr1 = self.fgr1body.getPosition()
        # rotfgr1= self.fgr1body.getRotation()
        # matfgr1 = Mat4(rotfgr1)
        # matfgr1.setRow(3, posfgr1)
        # self.fgr1geom.setMat(matfgr1)

        self.simcontrolstart = time.time()

        taskMgr.doMethodLater(.1, self.addSmiley, "AddSmiley", extraArgs = [nobj], appendTask = True)
        taskMgr.add(self.updateODE, "UpdateODE")

        # pg.plotAxisSelf(self.base.render)

    def setupODE(self):
        self.odeworld = OdeWorld()
        self.odeworld.setGravity(0, 0, -9.81)

        self.odespace = OdeSimpleSpace()
        self.contacts = OdeJointGroup()

    def addGround(self):
        boxx = 500.0
        boxy = 500.0
        boxz = 1.0
        pg.plotBox(self.base.render, pos = [0,0,-1.0], x = boxx*2.0, y = boxy*2.0, z = boxz*2.0, rgba=None)
        groundGeom = OdePlaneGeom(self.odespace, Vec4(0, 0, 1, 0))


    def addSmiley(self, nobj, task):
        sm = base.render.attachNewNode("part")
        sm.setPos(random.uniform(-20, 20), random.uniform(-30, 30), 20.0+self.smileyCount*30.0)
        self.smiley.instanceTo(sm)

        body = OdeBody(self.odeworld)
        mass = OdeMass()
        mass.setSphereTotal(20, 1)
        body.setMass(mass)
        body.setPosition(sm.getPos())
        modelTrimesh = OdeTriMeshData(sm, True)
        modelGeom = OdeTriMeshGeom(self.odespace, modelTrimesh)
        modelGeom.setBody(body)

        sm.setPythonTag("body", body)
        self.smileyCount += 1
        self.partlist.append(sm)

        if self.smileyCount == nobj:
            return task.done
        return task.again

    def updateODE(self, task):
        simcontrol = time.time()
        if simcontrol - self.simcontrolstart < self.nobj*5.0:
            self.odespace.collide((self.odeworld, self.contacts), self.near_callback)
            self.odeworld.quickStep(globalClock.getDt())
        elif simcontrol - self.simcontrolstart < self.nobj*5.0 + 1.0:
            self.gripper.resethand()
            self.gripper.update()
        else:
            self.odespace.collide((self.odeworld, self.contacts), self.near_callback)
            self.odeworld.quickStep(globalClock.getDt())
            self.gripper.update()
            if self.gripper.gripProc():
                self.gripper.resethand()

        for smiley in self.partlist:
            body = smiley.getPythonTag("body")
            smiley.setPosQuat(body.getPosition(), Quat(body.getQuaternion()))

        self.contacts.empty()
        return task.cont

    # Collision callback
    def near_callback(self, args, geom1, geom2):
        """Callback function for the collide() method.

        This function checks if the given geoms do collide and
        creates contact joints if they do.
        """

        # Check if the objects do collide
        contacts = OdeUtil.collide(geom1, geom2)

        # Create contact joints
        world, contactgroup = args
        for cgeom in contacts.getContactGeoms():
            c = OdeContact()
            c.setGeom(cgeom)
            s = OdeSurfaceParameters()
            s.setBounce(0.0)
            s.setMu(10)
            # s.setSoftCfm(0.001)
            # s.setSoftErp(0.8)
            c.setSurface(s)
            j = OdeContactJoint(world, contactgroup, c)
            j.attach(geom1.getBody(), geom2.getBody())

if __name__=='__main__':

    base = pandactrl.World(camp=[0,0,550], lookatp=[0,0,0])

    this_dir, this_filename = os.path.split(__file__)
    objpath = Filename.fromOsSpecific(os.path.join(this_dir, "objects", "cshape.egg"))
    odesim = ODESim(base, objpath, nobj = 5)

    # objtrimesh = trimesh.load_mesh(objpath)
    # objnp = pandageom.packpandanp(objtrimesh.vertices,
    #                               objtrimesh.face_normals, objtrimesh.faces)
    # objnp.setColor(.7,.5,.3,1)
    # objnp.reparentTo(base.render)
    #
    # def updateshow(task):
    #     print task.delayTime
    #     if abs(task.delayTime-13) < 1:
    #         task.delayTime -= 12.85
    #     return task.again
    # taskMgr.doMethodLater(1, updateshow, "tickTask")

    # dcam = loader.loadShader("depthmap.sha")
    # # render everything through this camera and shader
    # base.render.setShader(dcam)
    # loadPrcFileData('', 'show-buffers 1')

    base.run()
