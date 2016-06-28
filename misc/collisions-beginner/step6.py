# -*- coding: utf-8 -*-
"""
Mouse Picking with pattern masking

by fabius astelix @2010-01-25

Level: BEGINNER

We push the step5.py snippet forward performing the mouse picking in a complex situation with lotta balls of two kinds: smileys and frowney where we must recognize who is smiley from who is frowney using the CollisionHandlerEvent handler we have known in the last steps.

NOTE If you won't find here some line of code explained, probably you missed it in the previous steps - if you don't find there as well though, or still isn't clear for you, browse at http://www.panda3d.org/phpbb2/viewtopic.php?t=7918 and post your issue to the thread.
"""
from direct.showbase.DirectObject import DirectObject
from pandac.PandaModules import CollisionHandlerEvent, CollisionNode, CollisionSphere, CollisionTraverser, BitMask32, CollisionRay

import direct.directbase.DirectStart
#** snippet support routines - off the tutorial part
import snipstuff

#=========================================================================
# Some scenographic stuff
#=========================================================================

#** some scenographic stuff (text, light etc)
splash=snipstuff.splashCard()
snipstuff.info.append("Masked Mouse Picking")
snipstuff.info.append("a collision setup for picking objects with the mouse pointer driscriminating an object from another")
snipstuff.info.append("Move the mouse pointer over the smiley to see it change color and click LMB to see a reaction.\n")
snipstuff.info_show()

base.disableMouse()

#=========================================================================
# Main
"""
To biefly sum up what happen next, we'll make interact an invisible collision ray with two kinds of INTO objects: smileys and frowneys. The FROM ray will generate collision events as soon as touch each INTO object moving the mouse pointer over, routing toward different function handlers depending on the object group the object belongs (smiley or frowney). All of this is efficiently achieved using tags and special collision patterns, using therefore a slightly different setup from what we used to see so far in previous steps.
"""
#=========================================================================

#** Collision events ignition
base.cTrav=CollisionTraverser()
collisionHandler = CollisionHandlerEvent()

#** Setting the ray collider - see step5.py for details
pickerNode=CollisionNode('mouseraycnode')
pickerNP=base.camera.attachNewNode(pickerNode)
pickerRay=CollisionRay()
pickerNode.addSolid(pickerRay)

#** This is new stuff: we set here a so called 'tag' for the ray - its purpose is to make the ray recognizable in a different event pattern matching situation from what we are used to use so far. Just note the first parameter is the main object grouping. See details below setting the patterns.
pickerNode.setTag('rays','ray1')
base.cTrav.addCollider(pickerNP, collisionHandler)

#** This function is used to create all our smileys in the scene - I won't get into the basic commands that should be clear if you passed by the previous steps.
def smileyMachine(n, pos):
  smileyModel = loader.loadModel('smiley')
  smileyModel.setName('Smiley#%d'%n)
  smileyModel.reparentTo(render)
  smileyModel.setPos(pos)
  smileyCollider = smileyModel.attachNewNode(CollisionNode('smileycnode%d'%n))
  smileyCollider.node().addSolid(CollisionSphere(0, 0, 0, 1))
  # we set here the tag to recognize later all the smileys - just note to remember later that the grouping it belongs is called 'balls' and the value tells it is 'smileys'.
  smileyCollider.setTag('balls', 'smiley')

#** as the above this is a dispenser for the frowney objects
def frowneyMachine(n, pos):
  frowneyModel = loader.loadModel('frowney')
  frowneyModel.setName('Frowney#%d'%n)
  frowneyModel.reparentTo(render)
  frowneyModel.setPos(pos)
  frowneyCollider = frowneyModel.attachNewNode(
    CollisionNode('frowneycnode%d'%n)
  )
  frowneyCollider.node().addSolid(CollisionSphere(0, 0, 0, 1))
  # as for the smileys, we set here the tag to recognize later all the frowneys - take note of the tag and its value
  frowneyCollider.setTag('balls', 'frowney')

#** With this routine we generate automatically the balls of both groups placing'em one close the other in the same row
for i in range(6):
  if i % 2: smileyMachine(i, (-6+(i*2.2), 25, 0))
  else: frowneyMachine(i, (-6+(i*2.2), 25, 0))

#** This rack of 4 functions will be summoned by the CollisionHandlerEvent object as soon as the ray collide or leave each ball (the ray pierce IN and OUT a ball):
# here when ray goes INTO a smiley...
def collideInSmiley(entry):
  np_into=entry.getIntoNodePath()
  np_into.getParent().setColor(.6, 0.5, 1.0, 1)

# ...here when ray goes OUT a smiley...
def collideOutSmiley(entry):
  global pickingEnabledOject

  np_into=entry.getIntoNodePath()
  np_into.getParent().setColor(1.0, 1.0, 1.0, 1)
  snipstuff.info_message("Left '%s'"%np_into.getParent().getName())
  pickingEnabledOject=None

# and the same goes for frowney:
def collideInFrowney(entry):
  np_into=entry.getIntoNodePath()
  np_into.getParent().setColor(0, 0, .6, 1)

def collideOutFrowney(entry):
  global pickingEnabledOject

  np_into=entry.getIntoNodePath()
  np_into.getParent().setColor(1.0, 1.0, 1.0, 1)
  snipstuff.info_message("Left '%s'"%np_into.getParent().getName())
  pickingEnabledOject=None

#** This function is called as long as the mouse pointer goes and keep over a ball
def collideAgainBalls(entry):
  global pickingEnabledOject

  # since this function is called constantly while the mousepointer is over the ball, with this condition check we'll change things just the first time we are over a different ball than before.
  if entry.getIntoNodePath().getParent() <> pickingEnabledOject:
    np_from=entry.getFromNodePath()
    np_into=entry.getIntoNodePath()
    snipstuff.info_message(
      "'%s' INTO '%s'!\nYou may now click the LMB" % (
        np_from.getName(), np_into.getName()
      )
    )
    # we store the object actually picked because later, clicking the mouse button, we would change its shape a little fo showup - so, since the entry passed by the function is the collider object and not the model, to have its reference we need to call getParent()
    pickingEnabledOject=np_into.getParent()

#** This function will be called clicking the left mouse button and will change the ball scale according to the pickingEnabledOject actually stored in pickingEnabledOject
def mouseClick(status):
  global pickingEnabledOject

  if pickingEnabledOject:
    if status == 'down':
      pickingEnabledOject.setScale(.9)
      snipstuff.info_message("You clicked '%s'!"%pickingEnabledOject.getName())

    if status == 'up':
      pickingEnabledOject.setScale(1.0)

#** This is a task function called each frame, where the collision ray position is syncronized with the mouse pointer position
def rayupdate(task):
  if base.mouseWatcherNode.hasMouse():
    mpos=base.mouseWatcherNode.getMouse()
    # this function will set our ray to shoot from the actual camera lenses off the 3d scene, passing by the mouse pointer position, making  magically hit what is pointed by it in the 3d space
    pickerRay.setFromLens(base.camNode, mpos.getX(),mpos.getY())
  return task.cont

#** Now the tricky part: we have here a particular kind of pattern that react firing a task event when a collider, tagged as 'rays', whatever the value is stored into, hit an object tagged as 'balls', no matter what value is stored into its tag. The resulting event strings sent to the panda3D event manager will be the result of the FROM collider (ray) and the tag value owned by the INTO object being hit (a ball), provided that was settled with a tag key 'balls'.
# That said, these two lines will catch all the events for either smiles and frowneys because we both tagged 'em as 'balls', for all IN events...
collisionHandler.addInPattern("%(rays)ft-into-%(balls)it")
# ...and here for the OUT events
collisionHandler.addOutPattern("%(rays)ft-out-%(balls)it")

#** To complicate things a little, this time we'll going to use the addAgainPattern method, that will raise an event while the mouse ponter is keeping over a ball of any group. Note that the 'ray_again_all' chunk will be used by the CollisionHandlerEvent to fire the event. See the related accept below.
collisionHandler.addAgainPattern(
  "ray_again_all%(""rays"")fh%(""balls"")ih"
)
""" Note that we could have been done the same using this form as well:

collisionHandler.addAgainPattern("%(rays)ft-again-%(balls)it")

but then we should have used 2 accepts like this:

DO.accept('ray1-again-smileys', collideAgainBalls)
DO.accept('ray1-again-frowney', collideAgainBalls)

instead of just one as we did below, and this don't hurt very much in this snippet cos' we got just 2 groups, but could be complicated if we need to use a lot more groups.

Another big thing to note is that we could have been done all of this using the masking technique (see step4.py).
"""

#** to manage the collision events need Directobject, because the chaining between the CollisionHandlerEvent handler and our two functions seen above, happens with the accept() function of a DirectObject instance
DO=DirectObject()

#** Let's manage the collision events: as you can see in this rack of 4 accepts, these are the resulting events generated by the patterns defined above. Now it's up to you to understand the why an the how comparing these with those above.
DO.accept('ray1-into-smiley', collideInSmiley)
DO.accept('ray1-out-smiley', collideOutSmiley)
DO.accept('ray1-into-frowney', collideInFrowney)
DO.accept('ray1-out-frowney', collideOutFrowney)
# the event here instead, will call the collideAgainBalls function handler while the mouse pointer keep over any ball, either of the smiley or the frowney groups - still, check above how we defined the pattern string to understand how we came to this.
DO.accept('ray_again_all', collideAgainBalls)

#** Storage for the object actually hovered by the mouse pointer
pickingEnabledOject=None

#** Here's how we interact with mouse clicks - see the mouseClick function above
DO.accept('mouse1', mouseClick, ['down'])
DO.accept('mouse1-up', mouseClick, ['up'])

#** And at the end of all, we start the task that continuously update the ray collider position and orientation while we move the mouse pointer
taskMgr.add(rayupdate, "updatePicker")

splash.destroy()
run()
