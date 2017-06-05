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
        self.__danglew = np.array([0,0,0])

    @property
    def pos(self):
        return self.__pos

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
    def danglew(self):
        return self.__danglew

    @danglew.setter
    def danglew(self, value):
        self.__danglew = value

def doEulerPhysics(rbd, dtime):
    globalinerten = np.dot(rbd.rotmat, np.dot(rbd.inertiatensor, np.transpose(rbd.rotmat)))
    globalinerteninv = np.dot(rbd.rotmat, np.dot(np.linalg.inv(rbd.inertiatensor), np.transpose(rbd.rotmat)))
    angularmomentum = np.dot(globalinerten, rbd.anglew)
    rbd.danglew = np.dot(globalinerteninv, (-np.cross(rbd.anglew, angularmomentum)))
    # print rbd.danglew
    print angularmomentum
    # print rbd.rotmat
    anglewvalue = np.linalg.norm(rbd.anglew)
    axis = rbd.anglew/anglewvalue
    # note rodrigues is in degree
    theta = math.degrees(anglewvalue*dtime)
    # rbd.rotmat = np.dot(pg.mat3ToNp(Mat4.rotateMat(theta, Vec3(axis[0], axis[1], axis[2])).getUpper3()), rbd.rotmat)
    # rbd.rotmat = np.dot(tf.rotation_matrix(theta*math.pi/180.0, axis)[:3, :3], rbd.rotmat)
    rbd.rotmat = np.dot(rm.rodrigues(axis, theta), rbd.rotmat)
    rbd.anglew = rbd.anglew + rbd.danglew * dtime

    return angularmomentum

if __name__=="__main__":
    import os
    import math
    from panda3d.core import *
    import pandaplotutils.pandageom as pg
    import pandaplotutils.pandactrl as pc
    base = pc.World(camp = [3000,0,3000], lookatp = [0,0,0])

    rbd = Rigidbody(pos = np.array([0,0,0]), rotmat = np.identity(3),
                    inertiatensor = np.array([[80833.3,0.0,0.0],[0.0,68333.3,0.0],[0.0,0.0,14166.7]]))
    rbd.anglew = np.array([1,1,1])

    this_dir, this_filename = os.path.split(__file__)
    model_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "models", "box.egg"))
    model = loader.loadModel(model_filepath)
    model.reparentTo(base.render)

    rbdnp = []
    framenp = []

    import csv
    file = open('xyzw.csv', 'wb')
    writer = csv.writer(file)

    def updateshow(rbd, rbdnp, framenp, task):
        for frame in framenp:
            frame.detachNode()

        dtime = 0.002
        angularmomentum = doEulerPhysics(rbd, dtime)
        model.setMat(pg.cvtMat4(rbd.rotmat))

        # writer.writerow([str(rbd.anglew[0]), str(rbd.anglew[1]), str(rbd.anglew[2])])
        writer.writerow([str(angularmomentum[0]), str(angularmomentum[1]), str(angularmomentum[2])])

        arrownp = pg.plotArrow(base.render, epos = rbd.anglew*500, thickness = 15)
        framenp.append(arrownp)

        if task.time > 10.0:
            writer.close()

        return task.again

    taskMgr.add(updateshow, 'updateshow', extraArgs=[rbd, rbdnp, framenp], appendTask=True)
    base.run()
