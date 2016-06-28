# -*- coding: utf-8 -*-
"""
Collisions: starting up

by fabius astelix @2010-01-24

Level: NOVICE

This is the first very step to roughly undestand the panda3d collison system principles, which are the main elements involved, how to settle them, what is a FROM COLLIDER and what is an INTO COLLIDER and their roles.
In the scene there are two balls - they are found by default in the panda3D package; they are just two textured spheres by 2 unit diameter.
In this snippet, clicking the left button and moving the mouse to reach the other ball will fire a collision event catched via the usual panda3d task handling.
"""
from pandac.PandaModules import CollisionHandlerQueue, CollisionNode, CollisionSphere, CollisionTraverser

import direct.directbase.DirectStart
#** snippet support routines - off the tutorial part
import snipstuff

#=========================================================================
# Scenographic stuff
#=========================================================================

#** some scenographic stuff (text, light etc)
splash=snipstuff.splashCard()
snipstuff.info.append("First Collision Setup")
snipstuff.info.append("a minimal snippet to set up a basic collision situation")
snipstuff.info.append("Keep left mouse button down and move smiley trying to touch frowney.\n\nLMB+move=move smiley")
snipstuff.info_show()
snipstuff.dlight.setColor((.0, .1, .0, 1))

#=========================================================================
# Main
"""
To biefly sum up what we're going to see below, we create 2 interacting objects: a FROM object (smiley) and a INTO object (frowney), the FROM, normally the moving object who provoke the collision first, will generate collision events as soon as touch the INTO object. These events will be managed and stored in a queue by the CollisionHandlerQueue collision handler object and our script snippet will take care to check this queue and read the events stored to process'em.
"""
#=========================================================================

#** First off, we assign a traverser and a collision queue handler objects - these are the basic objects which will drive the whole stuff. The CollisionTraverser is often assigned to the showbase member base.cTrav when we call DirectStart and will ease up things because the showbase will take care to roll the CollisionTraverser  traverse tasks. Be warned though that base.cTrav OPERATES IN 3D ONLY BECAUSE TRAVERSE JUST THE render NODEPATH. Don't forget it.
base.cTrav=CollisionTraverser()
collisionHandler = CollisionHandlerQueue()

#** This is where we define the heart collider that will trig the collision event against the smiley ball, therefore it is who'll stir up the event, named FROM object. The major difference from this object and the other, called INTO object. is that we'll going to put it in the main traverser list, making it automatically figure as a FROM object just because of this. Any other object in the scene instead, is automatically considered as INTO by the system - the FROM either.

# first off we load the model as usual...
smileyModel = loader.loadModel('smiley')
smileyCollider = smileyModel.attachNewNode(CollisionNode('smileycnode'))
smileyCollider.node().addSolid(CollisionSphere(0, 0, 0, 1))
smileyModel.reparentTo(render)
smileyModel.setPos(-2, 25,0)
# ...and then we add it to the panda collision routines telling also that this is handled by the collision handler defined above. After this command the smiley becomes a FROM collider object.
base.cTrav.addCollider(smileyCollider, collisionHandler)

#** This is the frowney model that will intereact with the smiley - it is shaped and settled exactly as smiley so there isn't much to say here, other than this time we won't add this object to the main traverser. This makes him an INTO object.
# first we load the model as usual...
frowneyModel = loader.loadModel('frowney')
frowneyModel.reparentTo(render)
frowneyModel.setPos(2, 25,0)
#** ...then we set the collision geometry; we need first a CollisionNode...
frowneyCollider = frowneyModel.attachNewNode(CollisionNode('frowneycnode'))
#...then we add to that our CollisionSphere geometry primitive.
frowneyCollider.node().addSolid(CollisionSphere(0, 0, 0, 1))

# to make the stuff move without complicate routines, we just parent it to the camera so moving the camera with the mouse automatically moves frowney, but faking the effect as it was moving smiley.
frowneyModel.reparentTo(base.camera)

#** This is the loop periodically checked to find out if the have been collisions - it is fired by the taskMgr.add function set below.
def traverseTask(task=None):
  # as soon as a collison is detected, the collision queue handler will contain all the objects taking part in the collison, but we must sort that list first, so to have the first INTO object collided then the second and so on. Of course here it is pretty useless 'cos there is just one INTO object to collide with in the scene but this is the way to go when there are many other.
  collisionHandler.sortEntries()
  for i in range(collisionHandler.getNumEntries()):
    # we get here the n-th object collided (we know it is frowney for sure) - it is a CollisionEntry object (look into the manual to see its methods)
    entry = collisionHandler.getEntry(i)
    # we'll turn on the lights, to visually show this happy event
    snipstuff.dlight.setColor((.5, .5, .5, 1))
    snipstuff.info_message("Smiley is now touching Frowney!")
    # and we skip out cos we ain't other things to do here.
    if task: return task.cont

  # If there are no collisions the collision queue will be empty so the program flow arrives here and we'll shut down the lights and clear the text message
  snipstuff.dlight.setColor((.0, .1, .0, 1))
  snipstuff.info_message("")
  if task: return task.cont

#** let start the collision check loop
taskMgr.add(traverseTask, "tsk_traverse")

#** now we may start the show
splash.destroy()
run()
