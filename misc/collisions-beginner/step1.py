# -*- coding: utf-8 -*-
"""
Collisions: first contact(s)

by fabius astelix @2010-01-24

Level: NOVICE

In this step we're going to make something quite similar the first one but with one major difference: our INTO object won't be frowney anymore but rather another object which we define its collision geometries stright into the model editor of our choice, Blender3D.
In this step's scene you'll see then a ball and a heart - clicking the left button and moving the mouse to reach the ball will fire a collision.
Beside there are egg models ready to go, I suggest to open the models.blend file of this package and check what's inside, trying also to export the two models from scratch to see how it works the whole thing.
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
snipstuff.info.append("Simple Collision Setup")
snipstuff.info.append("a minimal snippet to show a simple collision situation")
snipstuff.info.append("Keep left mouse button down and move the smiley trying to touch the heart.\n\nLMB+move=move smiley")
snipstuff.info_show()
snipstuff.dlight.setColor((.0, .1, .0, 1))

#=========================================================================
# Main
"""
To biefly sum up what will happen below, we create 2 interacting objects: a FROM object (heart shape) and a INTO object (smiley), the FROM, normally the moving object who provoke the collision first, will generate collision events as soon as touch the INTO object. These events will be managed and stored in a queue by the CollisionHandlerQueue collision handler object and our script snippet will take care to read this queue and read the events stored to process'em.
"""
#=========================================================================

#** First off, we assign a traverser and a collision queue handler objects - these are the basic objects who drive the whole stuff. The CollisionTraverser is often assigned to the showbase member base.cTrav to ease up things because the showbase will take care to roll CollisionTraverser tasks
base.cTrav=CollisionTraverser()
collisionHandler = CollisionHandlerQueue()

#** This is where we define the heart collider that will trig the collision event against the smiley ball, therefore is who'll stir up the event.
# first off we load the model as usual - remember that panda3D will silently load the collider geometry as well but will be made automatically invisible
heartModel = loader.loadModel('heart')
heartModel.reparentTo(render)
heartModel.setPos(2, 25,0)
# to make the heart move without complicate routines, we just parent it to the camera so moving the camera with the mouse automatically moves the heart
heartModel.reparentTo(base.camera)

#** Here the important part: to have collsions, the engine may use the whole model mesh geometry, but especially for high-poly meshes this will result as a system overburden therefore, to lighten the system load is common practice to use simplified geometries instead - in our heart egg file we provided a sphere to use for this purpose: look into the file named 'models.blend' from the collisions-beginner folder (if you ain't so far) to know what I mean: you'll find there how is settled the collider to be automatically recognized by the model loader as a collider, without the hassle to do it here - btw, look the info text provided into the blender file to know where to find those settings,
# Now we get the collider reference off the heart model tree...
heartCollider = heartModel.find("**/collider_heart")
# ...and then we add it to the panda collision routines telling also that this is handled by the collision handler defined above
base.cTrav.addCollider(heartCollider, collisionHandler)

# as said somewhere else, the colliders defined into blender are made invisible automatically by the panda3D model loader but if you're curious to see it, uncomment this following line:
#heartCollider.show()


#** This is the smiley model that will intereact with the heart - this time we gotta model without a collision geometry in it, therefore we must create this collide geometry via scripting, using the panda3D CollisionSphere primitive, guess what shaped as a simple sphere. By the way this is not the only primitive panda3d could offer but it is recognized as the computationally faster  so the preferred one and in this case, since we gotta wrap it around a sphere, the perfect shape we may have. Browse the panda3D manual to know more of the other collision shapes.
# first we load the model as usual...
smileyModel = loader.loadModel('smiley')
smileyModel.reparentTo(render)
smileyModel.setPos(-2, 25,0)
#** ...then we set the collision geometry; we need first a CollisionNode...
smileyCollider = smileyModel.attachNewNode(CollisionNode('smileycnode'))
#...then we add to that our CollisionSphere geometry primitive - the parameters you see settled are the origin in the first three and the sphere radius with the last one: we set this 1 unit radius, 'cos our smiley is big as much.
smileyCollider.node().addSolid(CollisionSphere(0, 0, 0, 1))

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

  # If there are no collisions the collision queue will be empty so the program flow arrives here and we'll shut down the lights
  snipstuff.dlight.setColor((.0, .1, .0, 1))
  snipstuff.info_message("")
  if task: return task.cont

#** let start the collision check loop, called each 20th of second
taskMgr.add(traverseTask, "tsk_traverse")

splash.destroy()
run()
