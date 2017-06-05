from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletBoxShape
from panda3d.bullet import BulletTriangleMesh
from panda3d.bullet import BulletTriangleMeshShape
from panda3d.bullet import BulletDebugNode
from panda3d.bullet import BulletRigidBodyNode
from panda3d.core import *
import pandaplotutils.pandactrl as pc
import pandaplotutils.pandageom as pg
import utils.collisiondetection as cd
import os

base = pc.World(camp = [3000,0,3000], lookatp = [0,0,0])

# shape = BulletBoxShape(Vec3(50, 200, 450))
# node = BulletRigidBodyNode('Box')
# node.setMass(1.0)
# node.setAngularVelocity(Vec3(1,1,1))
# node.addShape(shape)

# np = base.render.attachNewNode(node)
# np.setPos(0, 0, 0)
this_dir, this_filename = os.path.split(__file__)
model_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "models", "top2.egg"))
model = loader.loadModel(model_filepath)
rbdnodepath = base.render.attachNewNode("topnode")
model.instanceTo(rbdnodepath)
rbdnodepath.setMat(Mat4.rotateMat(10, Vec3(1,0,0)))
rbdnodepath.setPos(0,0,300)

topbullnode = cd.genCollisionMeshNp(rbdnodepath)
topbullnode.setMass(.5)
topbullnode.setLinearVelocity(Vec3(0,0,0))
# topbullnode.setAngularVelocity(rbdnodepath.getMat().getRow3(2)*50.0)
# topbullnode.applyTorqueImpulse(rbdnodepath.getMat().getRow3(2)*50000.0)
topbullnode.applyTorqueImpulse(Vec3(0,0,1)*50000.0)
print topbullnode.getInertia()

world = BulletWorld()
world.setGravity(Vec3(0, 0, 0))
world.attachRigidBody(topbullnode)
pg.plotAxisSelf(base.render)

def update(task):
    dt = globalClock.getDt()
    world.doPhysics(dt)
    vecw= topbullnode.getAngularVelocity()
    arrownp = pg.plotArrow(base.render, epos = vecw*1000, thickness = 15)
    # print rotmat
    topbullnode.get

    return task.cont

debugNode = BulletDebugNode('Debug')
debugNode.showWireframe(True)
debugNode.showConstraints(True)
debugNode.showBoundingBoxes(False)
debugNode.showNormals(True)
debugNP = base.render.attachNewNode(debugNode)
debugNP.show()
world.setDebugNode(debugNP.node())

taskMgr.add(update, 'update')
base.run()