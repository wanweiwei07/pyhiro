# -*- coding: utf-8 -*-
"""
Collision events

by fabius astelix @2010-01-24

Level: BEGINNER

This setup is quite similar as in step1.py but use a different approach or, better, a different collision handler object: CollisionHandlerEvent. This handler uses a cleaner and more efficient way to detect when the objects collide and when they leaves each other, engaging python scripting just when needed therefore this is the preferred and faster way to manage colisions.

NOTE If you won't find here some line of code explained, probably you missed it in the previous steps - if you don't find there as well though, or still isn't clear for you, browse at http://www.panda3d.org/phpbb2/viewtopic.php?t=7918 and post your issue to the thread.
"""
from pandac.PandaModules import CollisionHandlerEvent, CollisionNode, CollisionSphere, CollisionTraverser
from direct.showbase.DirectObject import DirectObject

import direct.directbase.DirectStart
#** snippet support routines - off the tutorial part
import snipstuff

#=========================================================================
# Scenographic stuff
#=========================================================================

#** some scenographic stuff (text, light etc)
splash=snipstuff.splashCard()
snipstuff.info.append("Collision events")
snipstuff.info.append("a minimal snippet to show event collisions")
snipstuff.info.append("Keep left mouse button down and move the smiley, trying to reach the heart.\n\nLMB+move=move smiley")
snipstuff.info_show()
snipstuff.dlight.setColor((.0, .1, .0, 1))

#=========================================================================
# Main
"""
To biefly sum up what will happen below, we create 2 interacting objects: a FROM object (heart shape) and a INTO object (smiley), the FROM will generate collision events as soon as touch the INTO object. These events will be directly routed to user defined functions or method handlers by the CollisionHandlerEvent object.
"""
#=========================================================================

#** let's ignite the collision system as usual and declare the main handler
base.cTrav=CollisionTraverser()
collisionHandler = CollisionHandlerEvent()

#** as seen before we load and settle our heart to collide - it is important to note that this will be the FROM collider because will be the one moving around and therefore will be the one acting as the starter object of the collision.
heartModel = loader.loadModel('heart')
heartModel.reparentTo(render)
heartModel.setPos(2, 25,0)
# this is just to move the heart - never mind it
heartModel.reparentTo(base.camera)
heartCollider = heartModel.find("**/collider_heart")
base.cTrav.addCollider(heartCollider, collisionHandler)

#** And now the smiley turn - note this will be the INTO object for what we said above. So far it is just the same setup as in step1.py...
smileyModel = loader.loadModel('smiley')
smileyModel.reparentTo(render)
smileyModel.setPos(-2, 25,0)
smileyCollider = smileyModel.attachNewNode(CollisionNode('smileycnode'))
smileyCollider.node().addSolid(CollisionSphere(0, 0, 0, 1))

#**...but from now on things are about to change: these two functions are the handlers the collision handler will call as soon as two events will happens: the two objects touch each other, the two objects leave each other alone. Afterwards we'll see how to make this happen.
# this is the function will be called while the two objects collide...
def collideEventIn(entry):
  # lights on
  snipstuff.dlight.setColor((.5, .5, .5, 1))
#... and this when they leave each other alone.
def collideEventOut(entry):
  # lights shut down
  snipstuff.dlight.setColor((.0, .1, .0, 1))

#** And here it is how we tell to our handler which are the two functions to call whenever a collision or the opposite event happens:
# this method is for the INTO event, as is when the FROM object (heart) collide with the INTO one (smiley). You see that we set a pattern string with strange symbols inside - I'm not going to dig these details cos you'll find a wide explanation inside the panda3D manual at the chapter "Collision Handlers" - just know that that pattern will match a system event we'll define later, which the collision handler will call ASA the event occurs.
collisionHandler.addInPattern('%fn-into-%in')

# this is on the other hand the relative call for the OUT event, as is when the FROM object (heart) goes OUT the INTO oject (heart).
collisionHandler.addOutPattern('%fn-out-%in')

#** To manage the collision events need Directobject, because the chaining between the CollisionHandlerEvent handler and our two functions seen above, happens with the accept() function of a DirectObject instance that define the event strings the task manager will call via the CollisionHandlerEvent stimulus.
DO=DirectObject()

#** And at last here it is the event catching setup: we tell the panda3D event manager to call our function handlers whenever 'somebody' fires the strings 'collider_heart-into-smileycnode' or 'collider_heart-out-smileycnode' - this is to say that whenever the CollisionHandlerEvent object detect a collision, will send an event string to the engine formatted, as you can see, with first the name of the FROM collider (collider_heart precisely) the "-into-" or "-out-" string chunk and at the end the name of the INTO collider (smileycnode precisely). Note that '-into-' and '-out-' are just for our convenience, nobody oblige us to write them like that: we may call'em 'dummy' and 'foo' respectively but provided that they're specified also in the pattern declaration above in place of '-into-' and '-out-'. Now go back above and compare what said here with the pattern strings specified above with the addInPattern and addOutPattern declarations and with a little help of the panda3D manual you should understand all this knotted stuff.
DO.accept('collider_heart-into-smileycnode', collideEventIn)
DO.accept('collider_heart-out-smileycnode', collideEventOut)

splash.destroy()
run()
