# -*- coding: utf-8 -*-
"""
Mouse Picking

by fabius astelix @2010-01-25

Level: BEGINNER

Mouse picking is the mouse pointer interaction with elements of the screen. We'll see below a little sample to know how to make mouse picking using the panda3d collision system.

NOTE If you won't find here some line of code explained, probably you missed it in the previous steps - if you don't find there as well though, or still isn't clear for you, browse at http://www.panda3d.org/phpbb2/viewtopic.php?t=7918 and post your issue to the thread.
"""
from direct.showbase.DirectObject import DirectObject
from pandac.PandaModules import CollisionHandlerEvent, CollisionNode, CollisionSphere, CollisionTraverser, BitMask32, CollisionRay

import direct.directbase.DirectStart
#** snippet support routines - off the tutorial part
import snipstuff

#=========================================================================
# Scenographic stuff
#=========================================================================

#** some scenographic stuff (text, light etc)
splash=snipstuff.splashCard()
snipstuff.info.append("Simple Mouse Picking")
snipstuff.info.append("a minimal sample to show a simple collision setup for picking an object with the mouse pointer")
snipstuff.info.append("move the mouse pointer over the smiley to see it change color and click LMB to see a reaction.\n")
snipstuff.info_show()

base.disableMouse()

#=========================================================================
# Main
"""
To biefly sum up what happen below, we create 2 interacting objects: a FROM object, an invisible collision ray, and a INTO object, smiley. The FROM will generate collision events as soon as touch the INTO object moving the mouse pointer over it.
"""
#=========================================================================

#** we settle the collision events as ususal
base.cTrav=CollisionTraverser()
collisionHandler = CollisionHandlerEvent()

#** Differently from the previous steps, we change here a little bit our main FROM collider, using now a strange kind of geometry: a ray - you may see it as a laser ray the constantly shoot from the camera lenses toward the infinite. The following lines will settle up all of this.
pickerNode=CollisionNode('mouseraycnode')
pickerNP=base.camera.attachNewNode(pickerNode)
pickerRay=CollisionRay()
pickerNode.addSolid(pickerRay)
base.cTrav.addCollider(pickerNP, collisionHandler)

#** Here we make our same old smiley as we saw many times so far
smileyModel = loader.loadModel('smiley')
smileyModel.reparentTo(render)
smileyModel.setPos(0, 25,0)
smileyCollider = smileyModel.attachNewNode(CollisionNode('smileycnode'))
smileyCollider.node().addSolid(CollisionSphere(0, 0, 0, 1))

#** This function will be called by the CollisionHandlerEvent object as soon as the ray will collide with smiley (the FROM ray pierce INTO the ball)
def collideEventIn(entry):
  global pickingEnabled

  # here how we get the references of the two colliding objects to show their names ASA this happen
  np_from=entry.getFromNodePath()
  np_into=entry.getIntoNodePath()
  snipstuff.info_message(
    "'%s' goes INTO '%s'!\nYou may now click the LMB" % (
      np_from.getName(), np_into.getName()
    )
  )
  np_into.getParent().setColor(.6, 0.5, 1.0, 1)

  # we need also to raise a flag to inform the mousePick routine that the picking is now active
  pickingEnabled=True

#** This function will be called as the ray will leave smiley (the FROM ray goes OUT the smiley)
def collideEventOut(entry):
  global pickingEnabled

  # now we update the flag to inform mousePick routine that the picking is actually no more
  pickingEnabled=False

  snipstuff.info_message("you LEFT smiley alone")
  np_into=entry.getIntoNodePath()
  np_into.getParent().setColor(1.0, 1.0, 1.0, 1)

#** This is the function called each frame by a task defined below to syncronize the shooting ray position with the mouse moving pointer.
def rayupdate(task):
  if base.mouseWatcherNode.hasMouse():
    mpos=base.mouseWatcherNode.getMouse()
    # this is what set our ray to shoot from the actual camera lenses off the 3d scene, passing by the mouse pointer position, making  magically hit in the 3d space what is pointed by it
    pickerRay.setFromLens(base.camNode, mpos.getX(),mpos.getY())
  return task.cont

#** This function then will be called when we click and release the left mouse button, showing himself changing the ball scale according with the pickingEnabled flag state.
def mousePick(status):
  if pickingEnabled:
    if status == 'down':
      smileyModel.setScale(.9)

  if status == 'up':
    smileyModel.setScale(1.0)

# Here as well we see something already seen in the previous steps related to collision events so I won't go into details.
collisionHandler.addInPattern('%fn-into-%in')
collisionHandler.addOutPattern('%fn-out-%in')

#** Let's manage now the collision events:
DO=DirectObject()
# if you went from step3 and step4, here should not be mysteries for you
DO.accept('mouseraycnode-into-smileycnode', collideEventIn)
DO.accept('mouseraycnode-out-smileycnode', collideEventOut)

#** This little but important variable will be used to know the picking state at any time - note that the picking is enabled ASA the mouse pointer goes above the ball and disabled when it leave.
pickingEnabled=False

#** This is how we interact with mouse clicks - see the mousePick function above for details
DO.accept('mouse1', mousePick, ['down'])
DO.accept('mouse1-up', mousePick, ['up'])

#** And at last, we start the task that continuously update the ray collider position and orientation while we move the mouse pointer
taskMgr.add(rayupdate, "updatePicker")

splash.destroy()
run()
