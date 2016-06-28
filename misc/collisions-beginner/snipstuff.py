# -*- coding: utf-8 -*-

import sys, time

from direct.showbase.DirectObject import DirectObject
from pandac.PandaModules import AmbientLight, DirectionalLight, PerspectiveLens, Spotlight
from pandac.PandaModules import TextNode, TransparencyAttrib, Vec3
from direct.task import Task
from direct.gui.OnscreenImage import OnscreenImage
from direct.gui.OnscreenText import OnscreenText
from direct.interval.IntervalGlobal import *

base.setFrameRateMeter(True)

info=[]
infotext={}
infotext['title']=OnscreenText(text = '', pos = (0, .92), scale = 0.08, mayChange=True, fg=(1,1,1,1), bg=(0,0,1,.7))
infotext['content']=OnscreenText(text = '', pos = (0, 0.84), scale = 0.05, mayChange=True, fg=(1,1,0,1), bg=(0,0,0,.5))
infotext['hint']=OnscreenText(text = '', pos = (-1.3, .0), scale = 0.05,
  mayChange=True, fg=(1,1,1,1), align=TextNode.ALeft, bg=(1, .3, 0, .6),
  wordwrap=15
)
infotext['message']=OnscreenText(text = '', pos = (0, -0.85), scale = 0.07, mayChange=True, fg=(1,1,1,1), bg=(1,0,0,.65))

collshow=False

#=====================================================================
#
def toggle_info():
  for it in infotext.values():
    if it.isHidden(): it.show()
    else: it.hide()

#=====================================================================
#
def toggle_collisions():
  global collshow
  collshow=not collshow
  if collshow:
    base.cTrav.showCollisions(base.render)
    l=base.render.findAllMatches("**/+CollisionNode")
    for cn in l: cn.show()
  else:
    base.cTrav.hideCollisions()
    l=base.render.findAllMatches("**/+CollisionNode")
    for cn in l: cn.hide()

#=====================================================================
#
def toggle_wire():
  base.toggleWireframe()
  base.toggleTexture()

#=====================================================================
#
def info_show():
  title,content,hint=info
  infotext['title'].setText(title)
  infotext['content'].setText(content)
  infotext['hint'].setText(
    "** HINTS **\n"+hint+"""
h=toogle infos
c=toggle collisions
x=toggle wireframe
ESC=exit
"""
  )

#=====================================================================
#
def info_message(message):
  if infotext['message'].getText() <> message:
    infotext['message'].setText(message)

#=====================================================================
#
def load3Dimage(imagepath):
  img=OnscreenImage(image=imagepath, pos = (0, 0, 0),
    parent=base.render, scale=(10.0)
  )
  img.setTransparency(TransparencyAttrib.MAlpha)
  return img

#=====================================================================
#
def load2Dimage(imagepath):
  img=OnscreenImage(image=imagepath, pos = (0, 0, 0),
    parent=base.render2d, scale=(.3,1.,.3)
  )
  img.setTransparency(TransparencyAttrib.MAlpha)
  return img

#=====================================================================
#
class splashCard(object):
  #------------------------------------------------------------
  #
  def __init__(self, loadingtex='loading.png'):
    self.myImage=load2Dimage(loadingtex)
    for i in range(3):
      base.graphicsEngine.renderFrame()
      time.sleep(0.3)
  #------------------------------------------------------------
  #
  def destroy(self): self.myImage.destroy()

#=====================================================================
#
class avatar_steer(DirectObject):
  """
  """
  def __init__(
    self, avatar, walk=None, jump=None, fwspeed=7., lefthand=False, steer2d=False
  ):
    self.avatar=avatar
    self.spd_forward=fwspeed
    self.steer2d=steer2d
    if self.steer2d: self.lockY=self.avatar.getY()
    else: self.lockY=None
    if callable(walk): self.walk=walk
    if callable(jump): self.jump=jump
    # True while avatar is in the air
    self.floating=False
    self.camdistY=abs(base.cam.getY())
    self.camdistZ=base.cam.getZ()-self.avatar.getZ()
    #setup keyboard
    if lefthand:
      self.kconf="up=arrow_up\ndown=arrow_down\nleft=arrow_left\nright=arrow_right\njump=rcontrol"
    else: self.kconf="up=w\ndown=s\nleft=a\nright=d\njump=space\n"
    self.keys = {}
    for kv in self.kconf.split(): self.acceptMultiKey(*kv.split('='))

  def start(self):
    # why priority = 35 ? see http://www.panda3d.org/phpbb2/viewtopic.php?p=508#508
    self.mainLoop = taskMgr.add(self.steerLoop, "steerLoop", priority = 35)
    self.mainLoop.last = 0

  def acceptMultiKey(self, val, key):
    self.keys[val]=0
    self.accept(key,self.setKey,[val, 1])
    self.accept(key+'-up',self.setKey,[val, 0])

  def setKey(self,key,val):
    self.keys[key]=val

  def keybControl(self,dt):
    move=self.spd_forward*dt
    vel=Vec3(0,0,0)

    if self.keys['up']: vel+=self.up(move)
    if self.keys['down']: vel+=self.down(move)
    if self.keys['right']: vel+=self.right(move)
    if self.keys['left']: vel+=self.left(move)

    ###self.avatar.setPos(self.avatar.getPos()+vel)
    self.avatar.setFluidPos(self.avatar.getPos()+vel)
    if self.lockY != None: self.avatar.setY(self.lockY)
    if (vel <> Vec3.zero()): self.walk(dt, vel)
    if self.keys['jump']: self.jump(dt)

  def down(self, move): return Vec3(0,-move,0)
  def up(self, move): return Vec3(0,move,0)
  def right(self, move): return Vec3(move,0,0)
  def left(self, move): return Vec3(-move,0,0)

  def walk(self, dt, vel): pass

  def jump(self, dt):
    if not self.floating:
      self.floating=True
      lf=LerpFunc(lambda z: self.avatar.setZ(z),
        fromData = self.avatar.getZ(),
        toData = self.avatar.getZ()+5.0, duration = 0.4,
        blendType = 'easeOut'
      )
      self.seq=Sequence(lf, Wait(.7))
      self.seq.start()
    elif not self.seq.isPlaying(): self.floating=False

  def steerLoop(self,task):
    dt = task.time - task.last
    task.last = task.time
    self.keybControl(dt)
    base.cam.setY(self.avatar.getY()-self.camdistY)
    if self.steer2d:
      base.cam.setX(self.avatar.getX())
      base.cam.setZ(self.avatar.getZ()+self.camdistZ)
    base.cam.lookAt(self.avatar)

    return Task.cont

#=========================================================================
# Main
#=========================================================================

alight = AmbientLight('alight')
alight.setColor((.3, .3, .3, 1))
alnp = render.attachNewNode(alight)
render.setLight(alnp)

dlight = DirectionalLight('dlight')
lv=.6
dlight.setColor((lv,lv,lv, 1))
dlnp = render.attachNewNode(dlight)
render.setLight(dlnp)
dlnp.setPos(0,-20,35)
dlnp.lookAt(0,0,0)

DO=DirectObject()
DO.accept('c', toggle_collisions)
DO.accept('h', toggle_info)
DO.accept('x', toggle_wire)
DO.accept('escape',sys.exit)
