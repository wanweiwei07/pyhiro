# TODO: reduce the dependency on panda3d

import math
import os

import utils.robotmath as rm
import pandaplotutils.pandactrl as pandactrl
import pandaplotutils.pandageom as pandageom
from direct.showbase.ShowBase import ShowBase
from panda3d.bullet import BulletDebugNode
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletTriangleMesh
from panda3d.bullet import BulletTriangleMeshShape
from panda3d.bullet import BulletWorld
from panda3d.core import *

from utils import designpattern

class Hrp5Three():
    '''
    use utils.designpattern.singleton() to get a single instance of this class

    the range of the hand is set to 0~82
    '''

    def __init__(self, jawwidth=82, hndid = 'lft'):
        '''
        load the hrp5three model, set jawwidth and return a nodepath
        the hrp5three gripper is composed of three 3-jnt fingers,
        the first jnt is assumed to be active
        its rotation range is set to 39.32~90 (angle between the inner surface of the finger and the palm)
        leading to 0~82 jawwidth

        ## input
        pandabase:
            the showbase() object
        jawwidth:
            the distance between the third jnts

        author: weiwei
        date: 20170316
        '''
        self.__hrp5threenp = NodePath("hrp5threehnd")
        self.__hrp5threefgrtippccnp = NodePath("hrp5threefgrtippcc")
        self.jawwidth = jawwidth
        self.hndid = hndid

        this_dir, this_filename = os.path.split(__file__)
        if hndid == 'lft':
            hrp5threebasepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5threeegg",
                                                                     "HRP-5P_ConceptDesign_Lhand_Link0_Plan2.egg"))
        elif hndid == 'rgt':
            hrp5threebasepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5threeegg",
                                                                     "HRP-5P_ConceptDesign_Rhand_Link0_Plan2.egg"))

        # leftfinger
        hrp5threelfinger0path = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5threeegg",
                                                                     "HRP-5P_ConceptDesign_Lfinger_Link0.egg"))
        hrp5threelfinger1path = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5threeegg",
                                                                     "HRP-5P_ConceptDesign_Lfinger_Link1.egg"))
        hrp5threelfinger2path = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5threeegg",
                                                                     "HRP-5P_ConceptDesign_Lfinger_Link2.egg"))
        # rightfinger
        hrp5threerfinger0path = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5threeegg",
                                                                     "HRP-5P_ConceptDesign_Rfinger_Link0.egg"))
        hrp5threerfinger1path = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5threeegg",
                                                                     "HRP-5P_ConceptDesign_Rfinger_Link1.egg"))
        hrp5threerfinger2path = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5threeegg",
                                                                     "HRP-5P_ConceptDesign_Rfinger_Link2.egg"))
        leftfgrpccpath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5threeegg",
                                                                 "HRP-5P_ConceptDesign_Lfinger_Tippcc.egg"))
        rightfgrpccpath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5threeegg",
                                                                 "HRP-5P_ConceptDesign_Rfinger_Tippcc.egg"))

        # loader is a global variable defined by panda3d
        hrp5three_basel = loader.loadModel(hrp5threebasepath)
        hrp5three_lfinger0 = loader.loadModel(hrp5threelfinger0path)
        hrp5three_lfinger1 = loader.loadModel(hrp5threelfinger1path)
        hrp5three_lfinger2 = loader.loadModel(hrp5threelfinger2path)
        hrp5three_rfinger0 = loader.loadModel(hrp5threerfinger0path)
        hrp5three_rfinger1 = loader.loadModel(hrp5threerfinger1path)
        hrp5three_rfinger2 = loader.loadModel(hrp5threerfinger2path)
        lfgrtippcc = loader.loadModel(leftfgrpccpath)
        rfgrtippcc = loader.loadModel(rightfgrpccpath)

        # base
        hrp5threebase = NodePath("hrp5threebase")
        hrp5three_basel.instanceTo(hrp5threebase)
        rotmat4 = Mat4.rotateMat(90, Vec3(0,1,0))
        hrp5threebase.setMat(hrp5threebase.getMat()*rotmat4)
        hrp5threebase.setPos(-102.7,0,0)

        if hndid == 'rgt':
            # lfgr
            hrp5threelfgr0 = NodePath("hrp5threelfgr0")
            hrp5three_lfinger0.instanceTo(hrp5threelfgr0)
            hrp5threelfgr0.setPos(0,47,-6.5)
            hrp5threelfgr0.reparentTo(hrp5threebase)
            hrp5threelfgr1 = NodePath("hrp5threelfgr1")
            hrp5three_lfinger1.instanceTo(hrp5threelfgr1)
            hrp5threelfgr1.setPos(0,46,0)
            hrp5threelfgr1.reparentTo(hrp5threelfgr0)
            hrp5threelfgr2 = NodePath("hrp5threelfgr2")
            hrp5three_lfinger2.instanceTo(hrp5threelfgr2)
            hrp5threelfgr2.setPos(0,27,0)
            hrp5threelfgr2.reparentTo(hrp5threelfgr1)
            # r0fgr upper-right one
            hrp5threer0fgr0 = NodePath("hrp5threer0fgr0")
            hrp5three_rfinger0.instanceTo(hrp5threer0fgr0)
            hrp5threer0fgr0.setPos(27,-47,-6.5)
            hrp5threer0fgr0.reparentTo(hrp5threebase)
            hrp5threer0fgr1 = NodePath("hrp5threer0fgr1")
            hrp5three_rfinger1.instanceTo(hrp5threer0fgr1)
            hrp5threer0fgr1.setPos(0,-46,0)
            hrp5threer0fgr1.reparentTo(hrp5threer0fgr0)
            hrp5threer0fgr2 = NodePath("hrp5threer0fgr2")
            hrp5three_rfinger2.instanceTo(hrp5threer0fgr2)
            hrp5threer0fgr2.setPos(0,-27,0)
            hrp5threer0fgr2.reparentTo(hrp5threer0fgr1)
            # r1fgr lower-right one
            hrp5threer1fgr0 = NodePath("hrp5threer1fgr0")
            hrp5three_rfinger0.instanceTo(hrp5threer1fgr0)
            hrp5threer1fgr0.setPos(-27,-47,-6.5)
            hrp5threer1fgr0.reparentTo(hrp5threebase)
            hrp5threer1fgr1 = NodePath("hrp5threer1fgr1")
            hrp5three_rfinger1.instanceTo(hrp5threer1fgr1)
            hrp5threer1fgr1.setPos(0,-46,0)
            hrp5threer1fgr1.reparentTo(hrp5threer1fgr0)
            hrp5threer1fgr2 = NodePath("hrp5threer1fgr2")
            hrp5three_rfinger2.instanceTo(hrp5threer1fgr2)
            hrp5threer1fgr2.setPos(0,-27,0)
            hrp5threer1fgr2.reparentTo(hrp5threer1fgr1)
            # fingertip pcc
            fgrtippccleft = NodePath("hrp5threerlfgrtippcc")
            lfgrtippcc.instanceTo(fgrtippccleft)
            lrot = fgrtippccleft.getMat()
            fgrtippccleft.setPos(lrot.getRow3(3)-lrot.getRow3(0)*172.7)
            # move rightfgr to +-27 along z axis
            fgrtippccright = NodePath("hrp5threerrfgrtippcc")
            subfgrtippccright0 = NodePath("hrp5threerrfgrtip0pcc")
            subfgrtippccright1 = NodePath("hrp5threerrfgrtip1pcc")
            rfgrtippcc.instanceTo(subfgrtippccright0)
            rfgrtippcc.instanceTo(subfgrtippccright1)
            subfgrtippccright0.setPos(subfgrtippccright0.getMat().getRow3(3)+subfgrtippccright0.getMat().getRow3(2)*27)
            subfgrtippccright1.setPos(subfgrtippccright1.getMat().getRow3(3)-subfgrtippccright1.getMat().getRow3(2)*27)
            subfgrtippccright0.reparentTo(fgrtippccright)
            subfgrtippccright1.reparentTo(fgrtippccright)
            rrot = fgrtippccright.getMat()
            fgrtippccright.setPos(rrot.getRow3(3)-rrot.getRow3(0)*172.7)

        if hndid == 'lft':
            # rfgr
            hrp5threerfgr0 = NodePath("hrp5threerfgr0")
            hrp5three_rfinger0.instanceTo(hrp5threerfgr0)
            hrp5threerfgr0.setPos(0,-47,-6.5)
            hrp5threerfgr0.reparentTo(hrp5threebase)
            hrp5threerfgr1 = NodePath("hrp5threerfgr1")
            hrp5three_rfinger1.instanceTo(hrp5threerfgr1)
            hrp5threerfgr1.setPos(0,-46,0)
            hrp5threerfgr1.reparentTo(hrp5threerfgr0)
            hrp5threerfgr2 = NodePath("hrp5threerfgr2")
            hrp5three_rfinger2.instanceTo(hrp5threerfgr2)
            hrp5threerfgr2.setPos(0,-27,0)
            hrp5threerfgr2.reparentTo(hrp5threerfgr1)
            # l0fgr upper-left one
            hrp5threel0fgr0 = NodePath("hrp5threel0fgr0")
            hrp5three_lfinger0.instanceTo(hrp5threel0fgr0)
            hrp5threel0fgr0.setPos(27,47,-6.5)
            hrp5threel0fgr0.reparentTo(hrp5threebase)
            hrp5threel0fgr1 = NodePath("hrp5threel0fgr1")
            hrp5three_lfinger1.instanceTo(hrp5threel0fgr1)
            hrp5threel0fgr1.setPos(0,46,0)
            hrp5threel0fgr1.reparentTo(hrp5threel0fgr0)
            hrp5threel0fgr2 = NodePath("hrp5threel0fgr2")
            hrp5three_lfinger2.instanceTo(hrp5threel0fgr2)
            hrp5threel0fgr2.setPos(0,27,0)
            hrp5threel0fgr2.reparentTo(hrp5threel0fgr1)
            # l1fgr lower-left one
            hrp5threel1fgr0 = NodePath("hrp5threel1fgr0")
            hrp5three_lfinger0.instanceTo(hrp5threel1fgr0)
            hrp5threel1fgr0.setPos(-27,47,-6.5)
            hrp5threel1fgr0.reparentTo(hrp5threebase)
            hrp5threel1fgr1 = NodePath("hrp5threel1fgr1")
            hrp5three_lfinger1.instanceTo(hrp5threel1fgr1)
            hrp5threel1fgr1.setPos(0,46,0)
            hrp5threel1fgr1.reparentTo(hrp5threel1fgr0)
            hrp5threel1fgr2 = NodePath("hrp5threel1fgr2")
            hrp5three_lfinger2.instanceTo(hrp5threel1fgr2)
            hrp5threel1fgr2.setPos(0,27,0)
            hrp5threel1fgr2.reparentTo(hrp5threel1fgr1)
            # fingertip pcc
            # move leftfgr to +-27 along z axis
            fgrtippccleft = NodePath("hrp5threellfgrtippcc")
            subfgrtippccleft0 = NodePath("hrp5threellfgrtip0pcc")
            subfgrtippccleft1 = NodePath("hrp5threellfgrtip1pcc")
            lfgrtippcc.instanceTo(subfgrtippccleft0)
            lfgrtippcc.instanceTo(subfgrtippccleft1)
            subfgrtippccleft0.setPos(subfgrtippccleft0.getMat().getRow3(3)+subfgrtippccleft0.getMat().getRow3(2)*27)
            subfgrtippccleft1.setPos(subfgrtippccleft1.getMat().getRow3(3)-subfgrtippccleft1.getMat().getRow3(2)*27)
            subfgrtippccleft0.reparentTo(fgrtippccleft)
            subfgrtippccleft1.reparentTo(fgrtippccleft)
            lrot = fgrtippccleft.getMat()
            fgrtippccleft.setPos(lrot.getRow3(3)-lrot.getRow3(0)*172.7)
            fgrtippccright = NodePath("hrp5threelrfgrtippcc")
            rfgrtippcc.instanceTo(fgrtippccright)
            rrot = fgrtippccright.getMat()
            fgrtippccright.setPos(rrot.getRow3(3)-rrot.getRow3(0)*172.7)

        hrp5threebase.reparentTo(self.__hrp5threenp)
        fgrtippccleft.reparentTo(self.__hrp5threefgrtippccnp)
        fgrtippccright.reparentTo(self.__hrp5threefgrtippccnp)
        self.__hrp5threefgrtippccnp.setColor(.2,.2,.2,.2)
        self.__hrp5threefgrtippccnp.setTransparency(TransparencyAttrib.MAlpha)

        self.__jawwidthopen = 82.0
        self.__jawwidthclosed = 0.0
        self.setJawwidth(jawwidth)

    @property
    def handnp(self):
        # read-only property
        return self.__hrp5threenp

    @property
    def handpccnp(self):
        # read-only property
        return self.__hrp5threefgrtippccnp

    @property
    def jawwidthopen(self):
        # read-only property
        return self.__jawwidthopen

    @property
    def jawwidthclosed(self):
        # read-only property
        return self.__jawwidthclosed

    def setJawwidth(self, jawwidth):
        '''
        set the jawwidth of rtq85hnd
        the formulea is deduced on a note book
        the rtq85 gripper is composed of a parallelism and a fixed triangle,
        the parallelism: 1.905-1.905; 5.715-5.715; 70/110 degree
        the triangle: 4.75 (finger) 5.715 (inner knuckle) 3.175 (outer knuckle)

        ## input
        rtq85hnd:
            nodepath of a robotiq85hand
        jawwidth:
            the width of the jaw

        author: weiwei
        date: 20160627
        '''
        assert(jawwidth <= 82)
        assert(jawwidth >= 0)

        self.jawwidth = jawwidth
        fgrangle = self.__jawwidth2fgrangle()
        # right hand
        if self.hndid == "rgt":
            hrp5threelfgr0 = self.__hrp5threenp.find("**/hrp5threelfgr0")
            rotmat4 = Mat4.rotateMat(-fgrangle, Vec3(1,0,0))
            rotmat4.setRow(3,hrp5threelfgr0.getMat().getRow3(3))
            hrp5threelfgr0.setMat(rotmat4)
            hrp5threer0fgr0 = self.__hrp5threenp.find("**/hrp5threer0fgr0")
            rotmat4 = Mat4.rotateMat(fgrangle, Vec3(1,0,0))
            rotmat4.setRow(3,hrp5threer0fgr0.getMat().getRow3(3))
            hrp5threer0fgr0.setMat(rotmat4)
            hrp5threer1fgr0 = self.__hrp5threenp.find("**/hrp5threer1fgr0")
            rotmat4 = Mat4.rotateMat(fgrangle, Vec3(1,0,0))
            rotmat4.setRow(3,hrp5threer1fgr0.getMat().getRow3(3))
            hrp5threer1fgr0.setMat(rotmat4)
            fgrtippccleft = self.__hrp5threefgrtippccnp.find("**/hrp5threerlfgrtippcc")
            fgrtippccright = self.__hrp5threefgrtippccnp.find("**/hrp5threerrfgrtippcc")
            ltiprot = fgrtippccleft.getMat()
            rtiprot = fgrtippccright.getMat()
            fgrtippccleft.setPos(-ltiprot.getRow3(0)*172.7+ltiprot.getRow3(1)*(jawwidth)/2)
            fgrtippccright.setPos(-rtiprot.getRow3(0)*172.7-rtiprot.getRow3(1)*(jawwidth)/2)
        if self.hndid == "lft":
            hrp5threerfgr0 = self.__hrp5threenp.find("**/hrp5threerfgr0")
            rotmat4 = Mat4.rotateMat(fgrangle, Vec3(1,0,0))
            rotmat4.setRow(3,hrp5threerfgr0.getMat().getRow3(3))
            hrp5threerfgr0.setMat(rotmat4)
            hrp5threel0fgr0 = self.__hrp5threenp.find("**/hrp5threel0fgr0")
            rotmat4 = Mat4.rotateMat(-fgrangle, Vec3(1,0,0))
            rotmat4.setRow(3,hrp5threel0fgr0.getMat().getRow3(3))
            hrp5threel0fgr0.setMat(rotmat4)
            hrp5threel1fgr0 = self.__hrp5threenp.find("**/hrp5threel1fgr0")
            rotmat4 = Mat4.rotateMat(-fgrangle, Vec3(1,0,0))
            rotmat4.setRow(3,hrp5threel1fgr0.getMat().getRow3(3))
            hrp5threel1fgr0.setMat(rotmat4)
            fgrtippccleft = self.__hrp5threefgrtippccnp.find("**/hrp5threellfgrtippcc")
            fgrtippccright = self.__hrp5threefgrtippccnp.find("**/hrp5threelrfgrtippcc")
            ltiprot = fgrtippccleft.getMat()
            rtiprot = fgrtippccright.getMat()
            fgrtippccleft.setPos(-ltiprot.getRow3(0)*172.7+ltiprot.getRow3(1)*(jawwidth)/2.0)
            fgrtippccright.setPos(-rtiprot.getRow3(0)*172.7-rtiprot.getRow3(1)*(jawwidth)/2.0)

    def setPos(self, pandanpvec3):
        """
        set the pose of the hand
        changes self.__hrp5threenp

        :param pandanpvec3
        :return:
        """

        self.__hrp5threenp.setPos(pandanpvec3)

    def getPos(self):
        """
        get the pose of the hand

        :return:npvec3
        """

        return self.__hrp5threenp.getPos()

    def setMat(self, nodepath = None, pandanpmat4 = Mat4.identMat()):
        """
        set the absoluted translation and rotation of a robotiq hand
        with respect to nodepath
        changes self.rtq85np

        :param nodepath, the relative nodepath
        :param pandanpmat4: follows panda3d, a LMatrix4f matrix
        :return: null

        date: 20170612
        author: weiwei
        """

        if nodepath is not None:
            self.__hrp5threenp.setMat(nodepath, pandanpmat4)
        else:
            self.__hrp5threenp.setMat(pandanpmat4)

    def getMat(self):
        """
        get the rotation matrix of the hand

        :return: pandanpmat4: follows panda3d, a LMatrix4f matrix

        date: 20161109
        author: weiwei
        """

        return self.__hrp5threenp.getMat()

    def getHandName():
        return "hrp5three"

    def reparentTo(self, nodepath):
        """
        add to scene, follows panda3d

        :param nodepath: a panda3d nodepath
        :return: null

        date: 20161109
        author: weiwei
        """
        self.__hrp5threenp.reparentTo(nodepath)

    def removeNode(self):
        """

        :return:
        """

        self.__hrp5threenp.removeNode()

    def removeNode(self):
        """

        :return:
        """

        self.__hrp5threenp.removeNode()

    def lookAt(self, direct0, direct1, direct2):
        """
        set the Y axis of the hnd

        author: weiwei
        date: 20161212
        """

        self.__hrp5threenp.lookAt(direct0, direct1, direct2)
        self.__hrp5threefgrtippccnp.lookAt(direct0, direct1, direct2)

    def __jawwidth2fgrangle(self, jawwidth = None):
        """
        convert jawwidth to the angle of fingers
        the value is the angle between the inner surface of fingers and the surface of palm

        :param jawwidth, if it was not None, return the correspodent angle of self.jawwidth,
                else, return the correspondent angle of jawwidth
        :return: angle in degree

        author: weiwe
        date: 20170316
        """

        if jawwidth is not None:
            assert(jawwidth <= 82)
            assert(jawwidth >= 0)
            return 180-math.degrees(math.acos(float(82-jawwidth)/149.0))
        else:
            return 180-math.degrees(math.acos(float(82-self.jawwidth)/149.0))

    def xAlong(self, x, y, z):
        """
        set the X axis of the hnd along [x,y,z]

        author: weiwei
        date: 20170313
        """

        rotmat4z = Mat4.rotateMat(90, Vec3(0, 0, 1))

        self.__hrp5threenp.setMat(Mat4.identMat())
        self.__hrp5threenp.lookAt(x, y, z)
        self.__hrp5threenp.setMat(rotmat4z*self.__hrp5threenp.getMat())

        self.__hrp5threefgrtippccnp.setMat(Mat4.identMat())
        self.__hrp5threefgrtippccnp.lookAt(x, y, z)
        self.__hrp5threefgrtippccnp.setMat(rotmat4z*self.__hrp5threefgrtippccnp.getMat())


    def gripAt(self, fcx, fcy, fcz, c0nx, c0ny, c0nz, rotangle = 0, jawwidth = 82):
        '''
        set the hand to grip at fcx, fcy, fcz, fc = finger center
        the normal of the sglfgr contact is set to be c0nx, c0ny, c0nz
        the rotation around the normal is set to rotangle
        the jawwidth is set to jawwidth

        date: 20170316
        author: weiwei
        '''

        self.__hrp5threenp.setMat(Mat4.identMat())
        self.__hrp5threefgrtippccnp.setMat(Mat4.identMat())
        self.setJawwidth(jawwidth)
        if self.hndid == 'rgt':
            self.__hrp5threenp.lookAt(c0nx, c0ny, c0nz)
            self.__hrp5threefgrtippccnp.lookAt(c0nx, c0ny, c0nz)
        elif self.hndid == 'lft':
            self.__hrp5threenp.lookAt(-c0nx, -c0ny, -c0nz)
            self.__hrp5threefgrtippccnp.lookAt(-c0nx, -c0ny, -c0nz)
        rotmat4x = Mat4.rotateMat(rotangle, Vec3(c0nx, c0ny, c0nz))
        self.__hrp5threenp.setMat(self.__hrp5threenp.getMat()*rotmat4x)
        self.__hrp5threefgrtippccnp.setMat(self.__hrp5threefgrtippccnp.getMat()*rotmat4x)
        rotmat4 = Mat4(self.__hrp5threenp.getMat())
        handtipvec3 = rotmat4.getRow3(0)*172.7
        rotmat4.setRow(3, Vec3(fcx, fcy, fcz)+handtipvec3)
        self.__hrp5threenp.setMat(rotmat4)
        rotmat4 = Mat4(self.__hrp5threefgrtippccnp.getMat())
        handtipvec3 = rotmat4.getRow3(0)*172.7
        rotmat4.setRow(3, Vec3(fcx, fcy, fcz)+handtipvec3)
        self.__hrp5threefgrtippccnp.setMat(rotmat4)

def newHand(hndid = 'rgt', jawwidth = 82):
    return Hrp5Three(jawwidth= jawwidth, hndid = hndid)


if __name__=='__main__':

    import utils.collisiondetection as cd
    import pandaplotutils.pandageom as pg

    def updateworld(world, task):
        world.doPhysics(globalClock.getDt())
        # result = base.world.contactTestPair(bcollidernp.node(), lftcollidernp.node())
        # result1 = base.world.contactTestPair(bcollidernp.node(), ilkcollidernp.node())
        # result2 = base.world.contactTestPair(lftcollidernp.node(), ilkcollidernp.node())
        # print result
        # print result.getContacts()
        # print result1
        # print result1.getContacts()
        # print result2
        # print result2.getContacts()
        # for contact in result.getContacts():
        #     cp = contact.getManifoldPoint()
        #     print cp.getLocalPointA()
        return task.cont

    base = pandactrl.World()
    # first hand
    hrp5three = newHand(hndid='lft')
    hrp5three.gripAt(100,0,0,0,0,1,35,jawwidth = 30)
    hrp5three.reparentTo(base.render)
    hrp5three.handpccnp.reparentTo(base.render)
    handbullnp = cd.genCollisionMeshMultiNp(hrp5three.handnp)
    # second hand
    # hrp5three1 = newHand(hndid='rgt')
    # hrp5three1.reparentTo(base.render)
    # hrp5three1.fgrtippccleft.reparentTo(base.render)
    # hrp5three1.fgrtippccright.reparentTo(base.render)
    # hand1bullnp = cd.genCollisionMeshNp(hrp5three.handnp)

    pg.plotAxisSelf(base.render, Vec3(0,0,0))

    bullcldrnp = base.render.attachNewNode("bulletcollider")
    base.world = BulletWorld()

    base.taskMgr.add(updateworld, "updateworld", extraArgs=[base.world], appendTask=True)
    base.world.attachRigidBody(handbullnp)
    # result = base.world.contactTestPair(handbullnp, hand1bullnp)
    # for contact in result.getContacts():
    #     cp = contact.getManifoldPoint()
    #     print cp.getLocalPointA()
    #     pg.plotSphere(base.render, pos=cp.getLocalPointA(), radius=1, rgba=Vec4(1,0,0,1))
    #
    debugNode = BulletDebugNode('Debug')
    debugNode.showWireframe(True)
    debugNode.showConstraints(True)
    debugNode.showBoundingBoxes(False)
    debugNode.showNormals(False)
    debugNP = bullcldrnp.attachNewNode(debugNode)
    debugNP.show()

    base.world.setDebugNode(debugNP.node())

    base.run()