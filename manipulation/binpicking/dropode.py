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
import pandaplotutils.pandageom as pandageom
import trimesh
from utils import robotmath

from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
from panda3d.ode import *
import random

class ODESim(object):

    def __init__(self, base, objpath):
        self.base = base

        self.smiley = loader.loadModel(objpath)
        self.smileyCount = 0

        self.setupODE()
        self.addGround()

        self.partlist = []

        taskMgr.doMethodLater(1, self.addSmiley, "AddSmiley")
        taskMgr.add(self.updateODE, "UpdateODE")

    def setupODE(self):
        self.odeWorld = OdeWorld()
        self.odeWorld.setGravity(0, 0, -9.81)
        self.odeWorld.initSurfaceTable(1)
        self.odeWorld.setSurfaceEntry(0, 0, 200, 0.0, 9.1, 0.9,
                                    0.00001, 0.0, 0.002)

        self.space = OdeSimpleSpace()
        self.space.setAutoCollideWorld(self.odeWorld)
        self.contacts = OdeJointGroup()
        self.space.setAutoCollideJointGroup(self.contacts)

    def addGround(self):
        cm = CardMaker("ground")
        cm.setFrame(-100, 100, -100, 100)
        ground = self.base.render.attachNewNode(cm.generate())
        ground.setColor(0.2, 0.4, 0.8)
        ground.lookAt(0, 0, -1)
        groundGeom = OdePlaneGeom(self.space, Vec4(0, 0, 1, 0))

    def addSmiley(self, task):
        sm = base.render.attachNewNode("part")
        sm.setPos(random.uniform(-20, 20), random.uniform(-30, 30), random.uniform(50, 100))
        self.smiley.instanceTo(sm)

        body = OdeBody(self.odeWorld)
        mass = OdeMass()
        mass.setSphereTotal(20, 1)
        body.setMass(mass)
        body.setPosition(sm.getPos())
        modelTrimesh = OdeTriMeshData(sm, True)
        modelGeom = OdeTriMeshGeom(self.space, modelTrimesh)
        modelGeom.setBody(body)

        sm.setPythonTag("body", body)
        self.smileyCount += 1
        self.partlist.append(sm)

        if self.smileyCount == 10:
            return task.done
        return task.again

    def updateODE(self, task):
        self.space.autoCollide()
        self.odeWorld.quickStep(globalClock.getDt())

        for smiley in self.partlist:
            body = smiley.getPythonTag("body")
            smiley.setPosQuat(body.getPosition(), Quat(body.getQuaternion()))

        self.contacts.empty()
        return task.cont

if __name__=='__main__':

    base = pandactrl.World(camp=[0,0,550], lookatp=[0,0,0])

    this_dir, this_filename = os.path.split(__file__)
    objpath = Filename.fromOsSpecific(os.path.join(this_dir, "objects", "cshape.egg"))
    odesim = ODESim(base, objpath)

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

    dcam = loader.loadShader("depthmap.sha")
    # render everything through this camera and shader
    base.render.setShader(dcam)
    # loadPrcFileData('', 'show-buffers 1')

    base.run()
