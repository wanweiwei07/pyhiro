import numpy as np
import utils.robotmath as rm
import trimesh.transformations as tf
from panda3d.core import *

class Rigidbody(object):
    def __init__(self, pos = np.array([0,0,0]), rotmat = np.identity(3), inertiatensor = np.identity(3)):
        # note anglew must be in radian!
        # initialize a rigid body
        self.__name = 'rock'
        self.__pos = pos
        self.__rotmat = rotmat
        self.__inertiatensor = inertiatensor
        self.__anglew = np.array([0,0,0])
        self.__linearv = np.array([0,0,0])

    @property
    def pos(self):
        return self.__pos

    @pos.setter
    def pos(self, value):
        self.__pos = value

    @property
    def inertiatensor(self):
        return self.__inertiatensor

    @property
    def rotmat(self):
        return self.__rotmat

    @rotmat.setter
    def rotmat(self, value):
        self.__rotmat = value

    @property
    def anglew(self):
        return self.__anglew

    @anglew.setter
    def anglew(self, value):
        self.__anglew = value

    @property
    def linearv(self):
        return self.__linearv

    @linearv.setter
    def linearv(self, value):
        self.__linearv = value


def updateRbdVW(rbd, dtime):
    eps = 1e-6
    anglewvalue = np.linalg.norm(rbd.anglew)
    if anglewvalue < eps:
        rbd.pos = rbd.pos + dtime*rbd.linearv
        rbd.rotmat = rbd.rotmat
    else:
        theta = math.degrees(anglewvalue*dtime)
        waxis = rbd.anglew/anglewvalue
        vnormw = rbd.linearv/anglewvalue
        rotmat = rm.rodrigues(waxis, theta)
        rbd.pos = np.dot(rotmat, rbd.pos) + np.dot((np.identity(3)-rotmat), np.cross(waxis, vnormw)) + \
                    waxis.dot(waxis.transpose())*vnormw*anglewvalue*dtime
        rbd.rotmat = rotmat.dot(rbd.rotmat)

if __name__=="__main__":
    import os
    import math
    from panda3d.core import *
    import pandaplotutils.pandageom as pg
    import pandaplotutils.pandactrl as pc
    base = pc.World(camp = [30000,0,30000], lookatp = [0,0,0])

    rbd = Rigidbody(pos = np.array([0,0,0]), rotmat = np.identity(3),
                    inertiatensor = np.array([[80833.3,0.0,0.0],[0.0,68333.3,0.0],[0.0,0.0,14166.7]]))
    rbd.linearv = np.array([300,0,1000])
    rbd.anglew = np.array([1,0,0])

    this_dir, this_filename = os.path.split(__file__)
    model_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "models", "box.egg"))
    model = loader.loadModel(model_filepath)
    model.reparentTo(base.render)
    model.setMat(pg.cvtMat4(rbd.rotmat))
    model.setPos(pg.npToV3(rbd.pos))

    rbdnp = []
    framenp = []
    def updateshow(rbd, rbdnp, framenp, task):
        for frame in framenp:
            frame.detachNode()

        dtime = 0.02
        updateRbdVW(rbd, dtime)
        model = loader.loadModel(model_filepath)
        model.reparentTo(base.render)
        model.setMat(pg.cvtMat4(rbd.rotmat))
        model.setPos(pg.npToV3(rbd.pos))

        return task.again

    taskMgr.add(updateshow, 'updateshow', extraArgs=[rbd, rbdnp, framenp], appendTask=True)
    base.run()
