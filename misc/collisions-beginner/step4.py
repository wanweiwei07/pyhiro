# -*- coding: utf-8 -*-
"""
Masked Collision Events

by fabius astelix @2010-01-24

Level: BEGINNER

This is the same as step2.py but using event handlers.
Beside we got a more complex setup that double the elements involved, it follow the same principles as for step3.py plus the addition of masking as seen in step2.

NOTE If you won't find here some line of code explained, probably you missed it in the previous steps - if you don't find there as well though, or still isn't clear for you, browse at http://www.panda3d.org/phpbb2/viewtopic.php?t=7918 and post your issue to the thread.
"""
from direct.showbase.DirectObject import DirectObject
from pandac.PandaModules import CollisionHandlerEvent, CollisionNode, CollisionSphere, CollisionTraverser, BitMask32
from pandac.PandaModules import VBase4, AmbientLight, DirectionalLight
from direct.gui.OnscreenText import OnscreenText

import direct.directbase.DirectStart
#** snippet support routines - off the tutorial part
import snipstuff

#=========================================================================
# Scenographic stuff
#=========================================================================

#** some scenographic stuff (text, light etc)
splash=snipstuff.splashCard()
snipstuff.info.append("Masked Collision Events")
snipstuff.info.append("a snippet to show masked collision events")
snipstuff.info.append("Keep left mouse button down and move the balls, trying to reach different hearts.\n\nLMB+move=move the balls")
snipstuff.info_show()

#=========================================================================
# Main
"""
To biefly sum up what's going on below, we create 4 interacting objects: 2 FROM objects (heart shaped) and 2 INTO objects (ball shaped), the FROM will generate a collision event as soon as touch the INTO object with the same mask. Differently from step2.py we'll use the CollisionHandlerEvent as or main collision handler object and of course with a different setup.
"""
#=========================================================================

#** let's ignite the collision system as usual and declare the main handler
base.cTrav=CollisionTraverser()
collisionHandler = CollisionHandlerEvent()

#** the collision masks
LOVE_MASK=BitMask32.bit(1)
BROKEN_LOVE_MASK=BitMask32.bit(2)

#** As seen many times before we load and settle our heart to collide
heartModel = loader.loadModel('heart')
heartModel.reparentTo(render)
heartModel.setPos(0, 25, -2)
# this is just to move the heart - never mind it
heartModel.reparentTo(base.camera)
heartCollider = heartModel.find("**/collider_heart")
# let's mark it as a from collider and with its distinct mask as well
heartCollider.node().setFromCollideMask(LOVE_MASK)
base.cTrav.addCollider(heartCollider, collisionHandler)

#** and the same goes for the other heart but we'll put a different mask on it
brkheartModel = loader.loadModel('heart_broken')
brkheartModel.reparentTo(render)
brkheartModel.setPos(0, 25, 2)
brkheartModel.reparentTo(base.camera)
brkheartCollider = brkheartModel.find("**/collider_brkheart")
# here's its mask
brkheartCollider.node().setFromCollideMask(BROKEN_LOVE_MASK)
base.cTrav.addCollider(brkheartCollider, collisionHandler)

#** Now the INTO balls - first the smiley...
smileyModel = loader.loadModel('smiley')
smileyModel.reparentTo(render)
smileyModel.setPos(4, 25,0)
smileyModel.setColor(.4,.4,.4,1)
smileyCollider = smileyModel.attachNewNode(CollisionNode('smileycnode'))
smileyCollider.node().addSolid(CollisionSphere(0, 0, 0, 1))
# ...with the proper mask on...
smileyCollider.node().setIntoCollideMask(LOVE_MASK)

#** ...and the same goes for the frowney ball...
frowneyModel = loader.loadModel('frowney')
frowneyModel.reparentTo(render)
frowneyModel.setPos(-4, 25,0)
frowneyModel.setColor(.4,.4,.4,1)
frowneyCollider = frowneyModel.attachNewNode(CollisionNode('frowneycnode'))
frowneyCollider.node().addSolid(CollisionSphere(0, 0, 0, 1))
# ...with its own mask
frowneyCollider.node().setIntoCollideMask(BROKEN_LOVE_MASK)

#**...but from now on things are about to change: these two functions are the handlers the collision handler will call as soon as two events will happens: the two objects touch each other, the two objects leave each other alone.
# this is the function will be called while the two objects collide...
def collideEventIn(entry):
  # we retrieve the two object nodepaths - note that we need to go back the nodes hierarchy because the getXXXNodePath methods returns just the collision geometry, that we know is parented to the very object nodepath we need to manage here
  colliderFROM = entry.getFromNodePath().getParent()
  colliderINTO = entry.getIntoNodePath().getParent()
  # we now may change the aspect of the two colliding objects
  colliderINTO.setColor(1,1,1,1)
  colliderFROM.setScale(1.5)

#... and this when they leave each other alone.
def collideEventOut(entry):
  colliderFROM = entry.getFromNodePath().getParent()
  colliderINTO = entry.getIntoNodePath().getParent()
  colliderINTO.setColor(.4, .4, .4, 1)
  colliderFROM.setScale(1.0)

#** Here's the definition of the patterns that should catch our collisions - note that just these two are enough to catch the 2 couple of combinations.
collisionHandler.addInPattern('%fn-into-%in')
collisionHandler.addOutPattern('%fn-out-%in')

#** Let's manage the collision events:
DO=DirectObject()
# we should provide a couple of event strings for each couple of interacting objects (remember we made the smiley react against the whole heart and the frowney with the broken one). Compare these strings with the patterns above and if it is not enough to understand why it works, go back and dig again step2 and step3 until they're clear for you.
# Note that we route the in and out events to the same functions for both couples but that is just to slim the code.
DO.accept('collider_heart-into-smileycnode', collideEventIn)
DO.accept('collider_heart-out-smileycnode', collideEventOut)
#
DO.accept('collider_brkheart-into-frowneycnode', collideEventIn)
DO.accept('collider_brkheart-out-frowneycnode', collideEventOut)

splash.destroy()
run()
