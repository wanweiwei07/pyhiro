# -*- coding: utf-8 -*-
"""
Mouse Picking in 2D Space

by fabius astelix @2010-04-12

Level: BEGINNER

We come again with mouse picking as seen in steps 5 and 6 but this time we're going to make it in the flatter 2D space.

NOTE If you won't find here some line of code explained, probably you missed it in the previous steps - if you don't find there as well though, or still isn't clear for you, browse at http://www.panda3d.org/phpbb2/viewtopic.php?t=7918 and post your issue to the thread.
"""
import random

from direct.showbase.DirectObject import DirectObject
from pandac.PandaModules import CardMaker, CollisionHandlerEvent, CollisionNode, CollisionTraverser, BitMask32, CollisionRay

import direct.directbase.DirectStart
#** snippet support routines - off the tutorial part
import snipstuff

#=========================================================================
# Some scenographic stuff
#=========================================================================

#** some scenographic stuff (text, light etc)
splash=snipstuff.splashCard()
snipstuff.info.append("Mouse Picking in 2D Space")
snipstuff.info.append("a collision setup for picking objects in 2D")
snipstuff.info.append("Move the mouse pointer over a card and click it over.\n")
snipstuff.info_show()

base.disableMouse()

#=========================================================================
# Main
"""
While the principles are almost the same for 3D space, picking in 2D require a little change to adapt to the 2D layer, it's aspect and camera view (that is called orthographic).
"""
#=========================================================================

#** First big difference: as you may know, we used so far to assign the collision traverser to the global Showbase member cTrav, that is automatically updated (traversed) but it operates under the render nodepath that we know works in the 3D space and thewrefore we'll store the traverser apart in a convenient variable to be manually traversed later by our routine and act in 2d space.
customCtrav=CollisionTraverser()
collisionHandler = CollisionHandlerEvent()
# to keep the stuff simple, we specify fixed pattern strings to just check the into and out of all our colliders, because we know they are just cards and the ray FROM collider
collisionHandler.addInPattern("ray-into-cards")
collisionHandler.addOutPattern("ray-out-cards")

PICKING_MASK = BitMask32.bit(1)

#** Setting the ray collider
pickerNode=CollisionNode('mouseraycnode')
# another important but obvious difference from step 6 is that this time we parent the ray nodepath in render2d instead render root nodepath, otherwise all the objects we're going to define in 2D won't be seen by the ray.
pickerNP=base.render2d.attachNewNode(pickerNode)
pickerNP.show()
# note that this time we set the ray dimension along the Y axis 2 point long to pierce everything is on the Y=0 position (2D objects are usually placed there)
pickerRay=CollisionRay(0,-1,0, 0,1,0)
pickerNode.addSolid(pickerRay)
pickerNode.setFromCollideMask(PICKING_MASK)
pickerNode.setIntoCollideMask(BitMask32.allOff())
#** put the ray into the traverse cycle
customCtrav.addCollider(pickerNP, collisionHandler)

#** We create here 3 cards (indeed are) each of them tagged to bring with the id numnber we'll use to load the proper card texture later and its status which, in the beginning, is hidden (the back card facing up).
cm = CardMaker('cm')
left,right,bottom,top = 0, 1.4, 0, -2
cm.setFrame(left,right,top,bottom)
# Note we parent the card in aspect2d instead render2d because it will keep the right proportions even with different screen ratios
cardrootNP = aspect2d.attachNewNode('cardroot')
tex = loader.loadTexture('textures/cards/back.png')
for x in range(3):
  card = cardrootNP.attachNewNode(cm.generate())
  card.setPos(x*2, 0 , -1)
  card.setTag('id', str(x+1))
  card.setTag('status', 'hidden')
  card.setTexture(tex)
  card.setCollideMask(PICKING_MASK)
cardrootNP.setScale(.2)
b=(cardrootNP.getTightBounds()[1] - cardrootNP.getTightBounds()[0])
cardrootNP.setPos(-(b[0]/2.),0,(b[2]/2))

#** This 2 functions will be summoned by the CollisionHandlerEvent object as soon as the ray collide or leave each card

# here when ray goes INTO: we store the card nodepath pierced by the ray to eventually be used later after the user click it.
def collideInCard(entry):
  global pickingEnabledOject
  np_into=entry.getIntoNodePath()
  np_into.setScale(1.1)
  pickingEnabledOject=np_into

# ...here when ray goes OUT a card - we clear the storage because we're not above a card anymore with the mouse pointer.
def collideOutCard(entry):
  global pickingEnabledOject
  np_into=entry.getIntoNodePath()
  np_into.setScale(1)
  pickingEnabledOject=None

#** This function will be called clicking the left mouse button - if there is  an object reference stored in pickingEnabledOject, that is actually a card and so we're going to toggle its status and visibility from hidden to visible or vice versa
def mouseClick(mousebutton):
  global pickingEnabledOject
  if pickingEnabledOject:
    if mousebutton == 'up':
      cardid=pickingEnabledOject.getTag('id')
      status=pickingEnabledOject.getTag('status')
      if status == 'hidden':
        pickingEnabledOject.setTag('status', 'visible')
        tex = loader.loadTexture('textures/cards/%s.png'%cardid)
      else:
        pickingEnabledOject.setTag('status', 'hidden')
        tex = loader.loadTexture('textures/cards/back.png')
      pickingEnabledOject.setTexture(tex)
      snipstuff.info_message("You clicked the '%s' card!"%cardid)

#** This is a task function called each frame, where the collision ray position is syncronized with the mouse pointer position and, more important, we traverse our collision traverser to make its routines check the collisions.
def rayupdate(task):
  if base.mouseWatcherNode.hasMouse():
    mpos=base.mouseWatcherNode.getMouse()
    # here the meat: differently from step 6, we do not call setFromLens to shoot our ray in 3d space from the camera lens, but rather we move the ray along the cursor position and shoot a straight ray from there along Y axis. This is because in orthographic view there is no perspective and therefore a ray is shoot parallel to its starting point
    pickerNP.setPos(render2d,mpos[0],0,mpos[1])
    # and here we run the traverse step, to check the collisions in our scene
    customCtrav.traverse(render2d)
  return task.cont

#** to manage the collision and mouse button events
DO=DirectObject()

#** Let's manage the collision events: as you can see these are the events generated by the patterns defined above, a fixed string this time so no surprises here.
DO.accept('ray-into-cards', collideInCard)
DO.accept('ray-out-cards', collideOutCard)

#** Storage for the object eventually below the mouse pointer
pickingEnabledOject=None

#** Here's how we interact with mouse clicks - see the mouseClick function above
DO.accept('mouse1', mouseClick, ['down'])
DO.accept('mouse1-up', mouseClick, ['up'])

#** And here we start the task that continuously update the ray collider position and make traverse the collision traverser in 2d space
taskMgr.add(rayupdate, "updatePicker")

splash.destroy()
run()
