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
from panda3d.bullet import BulletDebugNode

import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pg
import trimesh
from utils import collisiondetection as cd

from direct.showbase.ShowBase import ShowBase
from panda3d.core import *
from panda3d.ode import *
import random

from panda3d.bullet import BulletBoxShape
from panda3d.bullet import BulletPlaneShape
from panda3d.bullet import BulletShape
from panda3d.bullet import BulletRigidBodyNode

class BulletSim(object):

    def __init__(self, base, objpath, bdebug = False):
        self.base = base

        self.smiley = loader.loadModel(objpath)
        self.smileyCount = 0

        self.setupBullet()
        self.addGround()
        # self.addWalls()

        self.partlist = []

        taskMgr.doMethodLater(.1, self.addSmiley, "AddSmiley")
        taskMgr.add(self.updateBlt, "UpdateBlt")

        if bdebug:
            debugNode = BulletDebugNode('Debug')
            debugNode.showWireframe(True)
            debugNode.showConstraints(True)
            debugNode.showBoundingBoxes(False)
            debugNode.showNormals(False)
            bullcldrnp = self.base.render.attachNewNode("bulletcollider")
            debugNP = bullcldrnp.attachNewNode(debugNode)
            debugNP.show()
            self.bltWorld.setDebugNode(debugNP.node())

    def setupBullet(self):
        self.bltWorld = BulletWorld()
        self.bltWorld.setGravity(Vec3(0, 0, -9.81))

    def addGround(self):
        boxx = 500.0
        boxy = 500.0
        boxz = 1.0
        shape = BulletBoxShape(Vec3(boxx, boxy, boxz)) # the parameters are half extends
        node = BulletRigidBodyNode('Ground')
        node.addShape(shape)
        self.bltWorld.attachRigidBody(node)
        pg.plotBox(self.base.render, pos = [0,0,-1.0], x = boxx*2.0, y = boxy*2.0, z = boxz*2.0, rgba=None)

        # shape = BulletPlaneShape(Vec3(0, 0, 1), 0.0)
        # node = BulletRigidBodyNode('Ground')
        # node.addShape(shape)
        # np = self.base.render.attachNewNode(node)
        # np.setPos(0, 0, 0)
        # self.bltWorld.attachRigidBody(node)

    def addWalls(self):
        """
        add walls

        :return:
        """

        # x+
        boxx = 2.0
        boxy = 50.0
        boxz = 100.0
        boxpx = 50.0
        boxpy = 0.0
        boxpz = 50.0
        shape = BulletBoxShape(Vec3(boxx, boxy, boxz)) # the parameters are half extends
        node = BulletRigidBodyNode('Wallx+')
        node.addShape(shape)
        node.setTransform(TransformState.makePos(VBase3(boxpx, boxpy, boxpz)))
        self.bltWorld.attachRigidBody(node)
        pg.plotBox(self.base.render, pos = [boxpx, boxpy, boxpz], x = boxx*2.0, y = boxy*2.0, z = boxz*2.0, rgba=None)
        # x-
        boxx = 2.0
        boxy = 50.0
        boxz = 100.0
        boxpx = -50.0
        boxpy = 0.0
        boxpz = 50.0
        shape = BulletBoxShape(Vec3(boxx, boxy, boxz)) # the parameters are half extends
        node = BulletRigidBodyNode('Wallx-')
        node.addShape(shape)
        node.setTransform(TransformState.makePos(VBase3(boxpx, boxpy, boxpz)))
        self.bltWorld.attachRigidBody(node)
        pg.plotBox(self.base.render, pos = [boxpx, boxpy, boxpz], x = boxx*2.0, y = boxy*2.0, z = boxz*2.0, rgba=None)
        # y+
        boxx = 50.0
        boxy = 2.0
        boxz = 100.0
        boxpx = 0.0
        boxpy = 50.0
        boxpz = 50.0
        shape = BulletBoxShape(Vec3(boxx, boxy, boxz))  # the parameters are half extends
        node = BulletRigidBodyNode('Wally+')
        node.addShape(shape)
        node.setTransform(TransformState.makePos(VBase3(boxpx, boxpy, boxpz)))
        self.bltWorld.attachRigidBody(node)
        pg.plotBox(self.base.render, pos=[boxpx, boxpy, boxpz], x=boxx * 2.0, y=boxy * 2.0, z=boxz * 2.0, rgba=None)
        # y-
        boxx = 50.0
        boxy = 2.0
        boxz = 100.0
        boxpx = 0.0
        boxpy = -50.0
        boxpz = 50.0
        shape = BulletBoxShape(Vec3(boxx, boxy, boxz))  # the parameters are half extends
        node = BulletRigidBodyNode('Wally-')
        node.addShape(shape)
        node.setTransform(TransformState.makePos(VBase3(boxpx, boxpy, boxpz)))
        self.bltWorld.attachRigidBody(node)
        pg.plotBox(self.base.render, pos=[boxpx, boxpy, boxpz], x=boxx * 2.0, y=boxy * 2.0, z=boxz * 2.0, rgba=None)

        # shape = BulletPlaneShape(Vec3(0, 0, 1), 0.0)
        # node = BulletRigidBodyNode('Ground')
        # node.addShape(shape)
        # np = self.base.render.attachNewNode(node)
        # np.setPos(0, 0, 0)
        # self.bltWorld.attachRigidBody(node)
        self.handz = 200.0

    def addSmiley(self, task):
        node = cd.genCollisionMeshNp(self.smiley)
        node.setMass(1.0)
        node.setName("part"+str(self.smileyCount))
        np = base.render.attachNewNode(node)
        np.setPos(random.uniform(-2, 2), random.uniform(-2, 2), 15.0+self.smileyCount*25.0)
        sm = np.attachNewNode("partrender"+str(self.smileyCount))
        self.smiley.instanceTo(sm)
        self.bltWorld.attachRigidBody(node)

        self.smileyCount += 1

        if self.smileyCount == 20:
            return task.done
        return task.again

    def updateBlt(self, task):
        self.bltWorld.doPhysics(globalClock.getDt())

        # nodep = self.base.render.find("**/part1")
        # if nodep:
        #     print nodep.getPos(), nodep.getHpr()
        # self.handz = self.handz-1
        # self.genHand(self.handz)

        return task.cont

    def genHand(self, handz=100.0):
        # fgr0
        boxx = 5.0
        boxy = 2.0
        boxz = 20.0
        boxpx = 0.0
        boxpy = 10.0
        boxpz = handz
        shape = BulletBoxShape(Vec3(boxx, boxy, boxz))  # the parameters are half extends
        node = BulletRigidBodyNode('fgr0')
        node.addShape(shape)
        node.setTransform(TransformState.makePos(VBase3(boxpx, boxpy, boxpz)))
        self.bltWorld.attachRigidBody(node)
        pg.plotBox(self.base.render, pos=[boxpx, boxpy, boxpz], x=boxx * 2.0, y=boxy * 2.0, z=boxz * 2.0, rgba=None)
        # fgr1
        boxx = 5.0
        boxy = 2.0
        boxz = 20.0
        boxpx = 0.0
        boxpy = -10.0
        boxpz = handz
        shape = BulletBoxShape(Vec3(boxx, boxy, boxz))  # the parameters are half extends
        node = BulletRigidBodyNode('fgr1')
        node.addShape(shape)
        node.setTransform(TransformState.makePos(VBase3(boxpx, boxpy, boxpz)))
        self.bltWorld.attachRigidBody(node)
        pg.plotBox(self.base.render, pos=[boxpx, boxpy, boxpz], x=boxx * 2.0, y=boxy * 2.0, z=boxz * 2.0, rgba=None)

if __name__=='__main__':

    base = pandactrl.World(camp=[0,0,550], lookatp=[0,0,0], fov = 20.0)

    this_dir, this_filename = os.path.split(__file__)
    objpath = Filename.fromOsSpecific(os.path.join(this_dir, "objects", "cshape.egg"))
    bltsim = BulletSim(base, objpath)

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
