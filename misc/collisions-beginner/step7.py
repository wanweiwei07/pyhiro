# -*- coding: utf-8 -*-
"""
inView method at work

by fabius astelix @2010-02-09

Level: BEGINNER

We introduce here a cute panda3d functionality that apparently ain't nothing to do with collisions, but that could be indeed used as it was and as a cheap way to do cool things like what we're going to show here: detect all the object in-view inside a lens frustum. After get this one, check in the advanced steps: there is a fancier showup to see the real potential of this feature.

NOTE If you won't find here some line of code explained, probably you missed it in the previous steps - if you don't find there as well though, or still isn't clear for you, browse at http://www.panda3d.org/phpbb2/viewtopic.php?t=7918 and post your issue to the thread.
"""
import random

import direct.directbase.DirectStart
#** snippet support routines - off the tutorial part
import snipstuff

#=========================================================================
# Scenographic stuff
#=========================================================================

base.cam.setY(-20)

#** some scenographic stuff (text, light etc)
splash=snipstuff.splashCard()
snipstuff.info.append("InView Method At Work")
snipstuff.info.append("how to use the inView method to detect if an object is\nin the view frustum of the camera")
snipstuff.info.append("LMB=start the frustum detection")
snipstuff.info_show()

#=========================================================================
# Main
"""
To biefly sum up what will happen below, we randomly spread around some smileys then, clicking the mouse, we'll detect and show all the smileys inside the frustum view of the main camera.
"""
#=========================================================================

#** random smiley generation - just plain and clean smileys, no collision or such.
spreadw=10
for i in range(10):
  ball = loader.loadModel("smiley")
  ball.reparentTo(render)
  ball.setPos(
    random.randint(-spreadw,spreadw), random.randint(-spreadw,spreadw),
    random.randint(-3,3)*2
  )

#** Clicking the mouse button will activate the camera in-sight smileys detection
def click():
  objects_inview=0
  objs=base.render.findAllMatches("**/smiley*")
  for o in objs:
    # here it is inView in action: for each smiley found in the whol;e scene we interrogate the inView method of the camera node to see if the smiley center is in the camera sight. Of course it is not very accurate like this, cos sometime the ball is not detected even if we partially see it in the screen  but it is enough to understand how it works this stuff
    if base.camNode.isInView(o.getPos(base.cam)):
      o.setColor((1,0,0,1))
      objects_inview+=1
    else: o.setColor((1,1,1,1))
  snipstuff.infotext['message'].setText("%d smileys in view"%objects_inview)

snipstuff.DO.accept("mouse1", click)

splash.destroy()
run()
