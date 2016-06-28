# -*- coding: utf-8 -*-
"""
Panda Physics - the panda3D simple physics engine

Level: BEGINNER

We're going now to start to discover what panda3d got to offer about this subject and how affect collision handling.

NOTE If you won't find here some line of code explained, probably you missed it in the previous steps - if you don't find there as well though, or still isn't clear for you, browse at http://www.panda3d.org/phpbb2/viewtopic.php?t=7918 and post your issue to the thread.
"""
import random
from direct.showbase.DirectObject import DirectObject
from pandac.PandaModules import *
from direct.interval.IntervalGlobal import *

loadPrcFileData("", """sync-video 0
"""
)
import direct.directbase.DirectStart
#** snippet support routines - not concerning the tutorial part
import snipstuff

#=========================================================================
# Scenographic stuff
#=========================================================================

base.cam.setPos(0,-30,10)
base.cam.lookAt((0,0,4))

splash=snipstuff.splashCard()
snipstuff.info.append("Panda Physics")
snipstuff.info.append("how setting up collisions to work with the panda3D physics engine")
snipstuff.info.append("r=object rain\n+-=increase/decrease dynamic friction\n*/=increase/decrease static friction\nwasd=move the avatar around\nSPACE=avatar jump")
snipstuff.info_show()

#=========================================================================
# Main
""" As you probably know, physics in games is a cool feature that give life to the show and today it is quite impossible not to face it while developing a modern game and panda3D make no exception. Though, you got to know that panda3d physics engine is not a complete physics engine but is there to solve simple tasks, especially related to particle management, even if could be applied to 3d models as well, so you may first understand what it got to offer and what are its limitations then see if suits for your game. We're not going to dig furter this issue, cos the panda3d manual already gotta dedicated section but rather we'll see how to setup collisions to interact with physics. We got a simple scene with a roaming smiley and a shower of frowneys and watermelons where each element in the scene will be able to collide each other, floor included and everything driven by the panda3d physics engine.
Note that we don't need to set masking in this setup cos each objet collide with each other, therefore every collider, beside the floor, gotta act as from and to object therefore the default masking setup is good as is.
"""
#=========================================================================

#** Collision system ignition - even if we're going to interact with the physics routines, the usual traverser is always in charge to drive collisions
base.cTrav=CollisionTraverser()
# look here: we enable the particle system - this is the evidence of what I was saying above, because the panda physics engine is conceived mainly to manage particles.
base.enableParticles()
# here there is the handler to use this time to manage collisions.
collisionHandler = PhysicsCollisionHandler()

#** This is the first time we see this collider: it is used mainly to define a flat infinite plane surface - it is very reliable and stable and gives high fame rate performances so use it as much as you can. Note that we specify 2 corners of the plane of just 1 unit per side but this collider will collide with everithing as it was an infinite plane anyhow. Note that there is no need to set anything further for it.
cp = CollisionPlane(Plane(Vec3(0, 0, 1), Point3(0, 0, 0)))
planeNP = base.render.attachNewNode(CollisionNode('planecnode'))
planeNP.node().addSolid(cp)
planeNP.show()

#** This is how to define the gravity force to make our stuff fall down: first off we define a ForceNode and attach to the render, so that everything below will be affected by this force
globalforcesFN=ForceNode('world-forces')
globalforcesFNP=base.render.attachNewNode(globalforcesFN)
# then we set a linear force that will act in Z axis-drag-down-force of 9.81 units per second.
globalforcesGravity=LinearVectorForce(0,0,-9.81)
globalforcesFN.addForce(globalforcesGravity)
# and then we assign this force to the physics manager. By the way, we never defined that manager, but it was made automatically when we called base.enableParticles()
base.physicsMgr.addLinearForce(globalforcesGravity)

#** Inside this function we'll load a model and assign a collide ball to it, suitable to collide with everything interacting in the physics environment. I put it in a function because I'll going to call it several times. It will return at last the topmost nodepath of its structure so that we may drive each ball around.
def phyball_dispenser(modelname, scale=1.):
  # first off we gotta define the topmost node that should be a PandaNode wrapped into a nodepath - this is mandatory cos if we try to directly use the  Actornode defined below, we'll face inexpected behavior manipulating the object.
  ballNP=NodePath(PandaNode("phisicsball"))
  # we then need an ActorNode - this is required when playing with physics cos it got an interface suitable for this task while the usual nodepath ain't. Then we'll stick it to the main nodepath we'll put into the scene render node, wrapped into a nodepath of course.
  ballAN=ActorNode("ballactnode")
  ballANP=ballNP.attachNewNode(ballAN)
  ballmodel=loader.loadModel(modelname)
  ballmodel.reparentTo(ballANP)
  ballmodel.setScale(scale)

  # as usual we need to provide a sphere collision node for our object. Note that the sphere collider is the only solid allowed to act either as FROM and  INTO object.
  ballCollider = ballANP.attachNewNode(CollisionNode('ballcnode'))
  ballCollider.node().addSolid(CollisionSphere(0,0,0, 1*scale))
  # now it's a good time to dip our object into the physics environment (the Actornode btw)
  base.physicsMgr.attachPhysicalNode(ballAN)
  # then tell to the PhysicsCollisionHandler what are its collider and main nodepath to handle - this means that the ballANP nodepath will be phisically moved to react to all the physics forces we applied in the environment (the gravity force in the specific). Note that due we are using a particular collison handler (PhysicsCollisionHandler) we cannot pass a common nodepath as we did in all the previous steps but a nodepath-wrapped Actornode.
  collisionHandler.addCollider(ballCollider, ballANP)
  # and inform the main traverser as well
  base.cTrav.addCollider(ballCollider, collisionHandler)
  # now the physic ball is ready to exit off the dispenser - Note we reparent it to render by default
  ballNP.reparentTo(base.render)
  return ballNP

#** let's call our dispenser made above to create a physics smiley
smiley=phyball_dispenser('smiley')

#** here is where will be produced the falling objects rain
def objectrain(howmany):
  def newfroney():
    phyball=phyball_dispenser(random.choice(['frowney','watermelon']))
    pos=(random.uniform(-1,1), random.uniform(-.5,.5), 30)
    phyball.setPos(pos)
  ivals=[]
  for i in range(howmany):
    ivals.append(Func(newfroney))
    ivals.append(Wait(.3))
  Sequence(*ivals).start()
snipstuff.DO.accept('r', objectrain, [30])

#** just an helper function - never mind
def acce(k,h,p):
  snipstuff.DO.accept(k, h, p)
  snipstuff.DO.accept(k+'-repeat', h, p)

#** just an helper function - never mind as well
def showmessage():
  snipstuff.infotext['message'].setText(
    ("Dynamic Friction=%0.4f\n"%collisionHandler.getDynamicFrictionCoef())+
    ("Static Friction=%0.4f"%collisionHandler.getStaticFrictionCoef())
  )

#** gui interface to play with the PhysicsCollisionHandler changing its default friction coefficients, just to see how the simulation changes like this.
def setdfc(d):
  v=max(0.01, min(1.0, collisionHandler.getDynamicFrictionCoef()+(.01*d)))
  collisionHandler.setDynamicFrictionCoef(v)
  showmessage()
acce('+', setdfc, [1])
acce('-', setdfc, [-1])

def setsfc(d):
  v=max(0.01, min(1.0, collisionHandler.getStaticFrictionCoef()+(.01*d)))
  collisionHandler.setStaticFrictionCoef(v)
  showmessage()
acce('*', setsfc, [1])
acce('/', setsfc, [-1])

#** Now we're ready to activate the avatar steering and go. Note how the jump routine changes in the physics environment: we add an 'impulse' as is a specific one-shot force - as in the real life works - and then it is the physics manager business to calculate the falling velocity and position during the simulation,
def avatarwalk(dt, vel):
  avatar=smiley.getChild(0).getChild(0)
  if vel[0]: avatar.setR(avatar.getR()+(40*vel[0]))
  if vel[1]: avatar.setP(avatar.getP()+(-38*vel[1]))
#
def avatarjump(dt):
  _smiley_actornode=smiley.getChild(0).node()
  if abs(_smiley_actornode.getPhysicsObject().getVelocity()[2]) < .01:
    _smiley_actornode.getPhysicsObject().addImpulse(Vec3(0,0,10))

steering=snipstuff.avatar_steer(
  smiley, avatarwalk, avatarjump, lefthand=False
)
steering.start()

#** let's start the show
splash.destroy()
run()
