# -*- coding: utf-8 -*-
"""
Simple collision setup with collision discrimination using the masking feature.

by fabius astelix @2010-01-24

Level: NOVICE

Starting from the simple setup in step1.py, we push things a little forward: now we got two colliders, a heart and a broken heart and two models with simple collision geometry to interact with, the smiley and frowney balls. To complicate things a little bit, one heart will be able to interact with just one of the two counterparts - you'll see that the smiley will interact just with the whole heart whereas the frowney ball just with the broken one - all of this would be possible using the collision masking feature explained below.

NOTE If you won't find here some line of code explained, probably you missed it in the previous steps - if you don't find there as well though, or still isn't clear for you, browse at http://www.panda3d.org/phpbb2/viewtopic.php?t=7918 and post your issue to the thread.
"""
from pandac.PandaModules import CollisionHandlerQueue, CollisionNode, CollisionSphere, CollisionTraverser, BitMask32

import direct.directbase.DirectStart
#** snippet support routines - off the tutorial part
import snipstuff

#=========================================================================
# Scenographic stuff
#=========================================================================

#** some scenographic stuff (text, light etc)
splash=snipstuff.splashCard()
snipstuff.info.append("Collision Masking")
snipstuff.info.append("a minimal collision setup that shows collision masking applied")
snipstuff.info.append("Keep left mouse button down and to move the balls, trying to reach different hearts.\n\nLMB+move=move the balls")
snipstuff.info_show()

#=========================================================================
# Main
"""
To biefly sum up what will happen below,we create 4 interacting objects: 2 FROM objects (heart shaped) and 2 INTO objects (ball shaped), the FROM will generate a collision event as soon as touch the INTO object with the same mask. As we did before, we'll use the CollisionHandlerQueue collision handler to orchestrate the collision operations.
"""
#=========================================================================

#** let's ignite the collision system as usual and declare the main handler - the important role of these two should already be clear to you, see step1.py if not
base.cTrav=CollisionTraverser()
collisionHandler = CollisionHandlerQueue()

#** First off we define here the collision masks - a mask is a special kind of integer that goes into range 0-32 that we'll use to discriminate from one object to another, like a cattle mark, therefore the number of masks we could define is 33 - we'll need just 2 so it is more than enough.
LOVE_MASK=BitMask32.bit(1)
BROKEN_LOVE_MASK=BitMask32.bit(2)

#** We'll load now the heart trigger - I don't go down into details you may find in step1.py
heartModel = loader.loadModel('heart')
# we exclude the heart geometry from all collisions so that it won't interfere with our following setups
heartModel.setCollideMask(BitMask32.allOff())
heartModel.reparentTo(render)
heartModel.setPos(0, 25, -2)
# this just make the heart move
heartModel.reparentTo(base.camera)
# as we know now here we get the collider reference off the heart model tree...
heartCollider = heartModel.find("**/collider_heart")
# ...but this time we'll assign it also a mask, to distinguish from the upcoming other heart  we'll define later
heartCollider.node().setFromCollideMask(LOVE_MASK)
# and now we're ready to add it to the collision system as usual
base.cTrav.addCollider(heartCollider, collisionHandler)

#** here how we settle the other heart - quite similar to the above but with a different mask
brkheartModel = loader.loadModel('heart_broken')
# we exclude this too from all collisions as we did for the other heart
brkheartModel .setCollideMask(BitMask32.allOff())
brkheartModel.reparentTo(render)
brkheartModel.setPos(0, 25, 2)
# this just make the heart move
brkheartModel.reparentTo(base.camera)
brkheartCollider = brkheartModel.find("**/collider_brkheart")
# here the masking
brkheartCollider.node().setFromCollideMask(BROKEN_LOVE_MASK)
# and now we're ready to add it to the collision system as well
base.cTrav.addCollider(brkheartCollider, collisionHandler)

#** Following will be defined the two iteracting balls, where we shall settle the smiling to interact just to the whole heart and the frowney just with the broken one. Let's start with smiley:
smileyModel = loader.loadModel('smiley')
smileyModel.reparentTo(render)
smileyModel.setPos(4, 25,0)
smileyCollider = smileyModel.attachNewNode(CollisionNode('smileycnode'))
smileyCollider.node().addSolid(CollisionSphere(0, 0, 0, 1))
# this is the magic line that tells smiley to react just against those objects marked with the LOVE_MASK - just simple like this
smileyCollider.node().setIntoCollideMask(LOVE_MASK)

#** and the same goes for the frowney ball, check it out:
frowneyModel = loader.loadModel('frowney')
frowneyModel.reparentTo(render)
frowneyModel.setPos(-4, 25,0)
frowneyCollider = frowneyModel.attachNewNode(CollisionNode('frowneycnode'))
frowneyCollider.node().addSolid(CollisionSphere(0, 0, 0, 1))
# here we tell frowney to react just against objects marked with BROKEN_LOVE_MASK
frowneyCollider.node().setIntoCollideMask(BROKEN_LOVE_MASK)

#** This is the loop periodically checked to find out if the have been collisions - it is fired by the taskMgr.doMethodLater function we'll set below.
def traverseTask(task=None):
  # as soon as a collison is detected, the collision queue handler will contain all the objects taking part in the collison - as we know we must put the list in order first.
  collisionHandler.sortEntries()
  for i in range(collisionHandler.getNumEntries()):
    # Differently from step1.py and to add more meat, we're going to use the useful getEntry() method of the CollisionHandlerQueue handler that allows us to get a reference of all the object colliders actually taking part the actual collisions
    entry = collisionHandler.getEntry(i)
    colliderNode = entry.getIntoNode()
    # now that we got the collider object reference, we may find out the name of who's involved in the current collision and therefore we'll going to change its appearance
    if colliderNode.getName() == 'smileycnode':
      smileyModel.setColor(1,1,1,1)
      heartModel.setScale(1.5)
      snipstuff.info_message("So much LOVE!")
      if task: return task.again
    elif colliderNode.getName() == 'frowneycnode':
      frowneyModel.setColor(1,1,1,1)
      brkheartModel.setScale(1.5)
      snipstuff.info_message("You broke my heart, sigh!")
      if task: return task.again

  # If there are no collisions the program flow arrives here and we'll reset the models appearance
  smileyModel.setColor(.4,.4,.4,1)
  frowneyModel.setColor(.4,.4,.4,1)
  heartModel.setScale(1.0)
  brkheartModel.setScale(1.0)
  snipstuff.info_message("")
  if task: return task.again

#** let start the collision check loop, called each 20th of second
taskMgr.doMethodLater(.2, traverseTask, "tsk_traverse")

splash.destroy()
run()
