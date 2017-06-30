import os

import numpy as np
from manipulation.grip.robotiq85 import rtq85
from manipulation.grip.hrp5three import hrp5three
from panda3d.core import *
import copy

import pandaplotutils.pandageom as pg

class Hrp2KMesh(object):
    """
    generate hrp2kmesh

    NOTE: it is unnecessary to attach a mnp to render repeatedly
    once attached, it is always there. update the joint angles will change the attached model directly
    """

    def __init__(self, handpkg):
        """
        load models

        :param handpkg:
        author: weiwei
        date: 20170612
        """

        self.__hrp5nmnp = NodePath("hrp5nmnp")

        ##########
        ### load the model files of the robots
        ##########
        this_dir, this_filename = os.path.split(__file__)

        # chest0-2, head1 (neck is not plotted)
        hrp5nbody_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Body.egg"))
        hrp5nchest0_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Chest_Link0.egg"))
        hrp5nchest1_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Chest_Link1.egg"))
        hrp5nchest2_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Chest_Link2.egg"))
        hrp5nhead0_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Head_Link0.egg"))
        hrp5nhead1_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Head_Link1.egg"))

        hrp5nbody_model = loader.loadModel(hrp5nbody_filepath)
        hrp5nchest0_model = loader.loadModel(hrp5nchest0_filepath)
        hrp5nchest1_model = loader.loadModel(hrp5nchest1_filepath)
        hrp5nchest2_model = loader.loadModel(hrp5nchest2_filepath)
        hrp5nhead0_model = loader.loadModel(hrp5nhead0_filepath)
        hrp5nhead1_model = loader.loadModel(hrp5nhead1_filepath)

        self.__hrp5nbody_nodepath = NodePath("hrp5nbody")
        self.__hrp5nchest0_nodepath = NodePath("hrp5nchest0")
        self.__hrp5nchest1_nodepath = NodePath("hrp5nchest1")
        self.__hrp5nchest2_nodepath = NodePath("hrp5nchest2")
        self.__hrp5nhead0_nodepath = NodePath("hrp5nhead0")
        self.__hrp5nhead1_nodepath = NodePath("hrp5nhead1")

        hrp5nbody_model.instanceTo(self.__hrp5nbody_nodepath)
        hrp5nchest0_model.instanceTo(self.__hrp5nchest0_nodepath)
        hrp5nchest1_model.instanceTo(self.__hrp5nchest1_nodepath)
        hrp5nchest2_model.instanceTo(self.__hrp5nchest2_nodepath)
        hrp5nhead0_model.instanceTo(self.__hrp5nhead0_nodepath)
        hrp5nhead1_model.instanceTo(self.__hrp5nhead1_nodepath)

        self.__hrp5nbody_nodepath.reparentTo(self.__hrp5nmnp)
        self.__hrp5nchest0_nodepath.reparentTo(self.__hrp5nbody_nodepath)
        self.__hrp5nchest1_nodepath.reparentTo(self.__hrp5nchest0_nodepath)
        self.__hrp5nchest2_nodepath.reparentTo(self.__hrp5nchest1_nodepath)
        self.__hrp5nhead0_nodepath.reparentTo(self.__hrp5nchest2_nodepath)
        self.__hrp5nhead1_nodepath.reparentTo(self.__hrp5nhead0_nodepath)

        # rgtarm
        hrp5nrgtarmlj0_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rscap_Link0.egg"))
        hrp5nrgtarmlj0_model = loader.loadModel(hrp5nrgtarmlj0_filepath)
        self.__hrp5nrgtarmlj0_nodepath = NodePath("nxtrgtarmlj0_nodepath")
        hrp5nrgtarmlj0_model.instanceTo(self.__hrp5nrgtarmlj0_nodepath)
        self.__hrp5nrgtarmlj0_nodepath.reparentTo(self.__hrp5nmnp)
        #
        hrp5nrgtarmlj1_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link0.egg"))
        hrp5nrgtarmlj1_model = loader.loadModel(hrp5nrgtarmlj1_filepath)
        self.__hrp5nrgtarmlj1_nodepath = NodePath("nxtrgtarmlj1_nodepath")
        hrp5nrgtarmlj1_model.instanceTo(self.__hrp5nrgtarmlj1_nodepath)
        self.__hrp5nrgtarmlj1_nodepath.reparentTo(self.__hrp5nmnp)
        #
        hrp5nrgtarmlj2_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link1.egg"))
        hrp5nrgtarmlj2_model = loader.loadModel(hrp5nrgtarmlj2_filepath)
        self.__hrp5nrgtarmlj2_nodepath = NodePath("nxtrgtarmlj2_nodepath")
        hrp5nrgtarmlj2_model.instanceTo(self.__hrp5nrgtarmlj2_nodepath)
        self.__hrp5nrgtarmlj2_nodepath.reparentTo(self.__hrp5nmnp)
        #
        hrp5nrgtarmlj3_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link2.egg"))
        hrp5nrgtarmlj3_model = loader.loadModel(hrp5nrgtarmlj3_filepath)
        self.__hrp5nrgtarmlj3_nodepath = NodePath("nxtrgtarmlj3_nodepath")
        hrp5nrgtarmlj3_model.instanceTo(self.__hrp5nrgtarmlj3_nodepath)
        self.__hrp5nrgtarmlj3_nodepath.reparentTo(self.__hrp5nmnp)
        #
        hrp5nrgtarmlj4_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link3.egg"))
        hrp5nrgtarmlj4_model = loader.loadModel(hrp5nrgtarmlj4_filepath)
        self.__hrp5nrgtarmlj4_nodepath = NodePath("nxtrgtarmlj4_nodepath")
        hrp5nrgtarmlj4_model.instanceTo(self.__hrp5nrgtarmlj4_nodepath)
        self.__hrp5nrgtarmlj4_nodepath.reparentTo(self.__hrp5nmnp)
        #
        hrp5nrgtarmlj5_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link4.egg"))
        hrp5nrgtarmlj5_model = loader.loadModel(hrp5nrgtarmlj5_filepath)
        self.__hrp5nrgtarmlj5_nodepath = NodePath("nxtrgtarmlj5_nodepath")
        hrp5nrgtarmlj5_model.instanceTo(self.__hrp5nrgtarmlj5_nodepath)
        self.__hrp5nrgtarmlj5_nodepath.reparentTo(self.__hrp5nmnp)
        #
        hrp5nrgtarmlj6_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link5.egg"))
        hrp5nrgtarmlj6_model = loader.loadModel(hrp5nrgtarmlj6_filepath)
        self.__hrp5nrgtarmlj6_nodepath = NodePath("nxtrgtarmlj6_nodepath")
        hrp5nrgtarmlj6_model.instanceTo(self.__hrp5nrgtarmlj6_nodepath)
        self.__hrp5nrgtarmlj6_nodepath.reparentTo(self.__hrp5nmnp)
        #
        hrp5nrgtarmlj7_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link6.egg"))
        hrp5nrgtarmlj7_model = loader.loadModel(hrp5nrgtarmlj7_filepath)
        self.__hrp5nrgtarmlj7_nodepath = NodePath("nxtrgtarmlj7_nodepath")
        hrp5nrgtarmlj7_model.instanceTo(self.__hrp5nrgtarmlj7_nodepath)
        self.__hrp5nrgtarmlj7_nodepath.reparentTo(self.__hrp5nmnp)

        # lftarm
        hrp5nlftarmlj0_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Lscap_Link0.egg"))
        hrp5nlftarmlj0_model = loader.loadModel(hrp5nlftarmlj0_filepath)
        self.__hrp5nlftarmlj0_nodepath = NodePath("nxtlftarmlj0_nodepath")
        hrp5nlftarmlj0_model.instanceTo(self.__hrp5nlftarmlj0_nodepath)
        self.__hrp5nlftarmlj0_nodepath.reparentTo(self.__hrp5nmnp)
        #
        hrp5nlftarmlj1_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link0.egg"))
        hrp5nlftarmlj1_model = loader.loadModel(hrp5nlftarmlj1_filepath)
        self.__hrp5nlftarmlj1_nodepath = NodePath("nxtlftarmlj1_nodepath")
        hrp5nlftarmlj1_model.instanceTo(self.__hrp5nlftarmlj1_nodepath)
        self.__hrp5nlftarmlj1_nodepath.reparentTo(self.__hrp5nmnp)
        #
        hrp5nlftarmlj2_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link1.egg"))
        hrp5nlftarmlj2_model = loader.loadModel(hrp5nlftarmlj2_filepath)
        self.__hrp5nlftarmlj2_nodepath = NodePath("nxtlftarmlj2_nodepath")
        hrp5nlftarmlj2_model.instanceTo(self.__hrp5nlftarmlj2_nodepath)
        self.__hrp5nlftarmlj2_nodepath.reparentTo(self.__hrp5nmnp)
        #
        hrp5nlftarmlj3_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link2.egg"))
        hrp5nlftarmlj3_model = loader.loadModel(hrp5nlftarmlj3_filepath)
        self.__hrp5nlftarmlj3_nodepath = NodePath("nxtlftarmlj3_nodepath")
        hrp5nlftarmlj3_model.instanceTo(self.__hrp5nlftarmlj3_nodepath)
        self.__hrp5nlftarmlj3_nodepath.reparentTo(self.__hrp5nmnp)
        #
        hrp5nlftarmlj4_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link3.egg"))
        hrp5nlftarmlj4_model = loader.loadModel(hrp5nlftarmlj4_filepath)
        self.__hrp5nlftarmlj4_nodepath = NodePath("nxtlftarmlj4_nodepath")
        hrp5nlftarmlj4_model.instanceTo(self.__hrp5nlftarmlj4_nodepath)
        self.__hrp5nlftarmlj4_nodepath.reparentTo(self.__hrp5nmnp)
        #
        hrp5nlftarmlj5_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link4.egg"))
        hrp5nlftarmlj5_model = loader.loadModel(hrp5nlftarmlj5_filepath)
        self.__hrp5nlftarmlj5_nodepath = NodePath("nxtlftarmlj5_nodepath")
        hrp5nlftarmlj5_model.instanceTo(self.__hrp5nlftarmlj5_nodepath)
        self.__hrp5nlftarmlj5_nodepath.reparentTo(self.__hrp5nmnp)
        #
        hrp5nlftarmlj6_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link5.egg"))
        hrp5nlftarmlj6_model = loader.loadModel(hrp5nlftarmlj6_filepath)
        self.__hrp5nlftarmlj6_nodepath = NodePath("nxtlftarmlj6_nodepath")
        hrp5nlftarmlj6_model.instanceTo(self.__hrp5nlftarmlj6_nodepath)
        self.__hrp5nlftarmlj6_nodepath.reparentTo(self.__hrp5nmnp)
        #
        hrp5nlftarmlj7_filepath = Filename.fromOsSpecific(
            os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link6.egg"))
        hrp5nlftarmlj7_model = loader.loadModel(hrp5nlftarmlj7_filepath)
        self.__hrp5nlftarmlj7_nodepath = NodePath("nxtlftarmlj7_nodepath")
        hrp5nlftarmlj7_model.instanceTo(self.__hrp5nlftarmlj7_nodepath)
        self.__hrp5nlftarmlj7_nodepath.reparentTo(self.__hrp5nmnp)

        # rgthnd
        self.hrp5nrobotrgthnd = handpkg.newHand('rgt')
        self.hrp5nrobotrgthnd.reparentTo(self.__hrp5nmnp)

        # lfthnd
        self.hrp5nrobotlfthnd = handpkg.newHand('lft')
        self.hrp5nrobotlfthnd.reparentTo(self.__hrp5nmnp)

        ##########
        ### load the model files of sticks
        ##########

        self.pggen = pg.PandaGeomGen()

    def gensnp(self, hrp5nrobot, rgtrbga=np.array([.5 ,0 ,0 ,0]), lftrgba=np.array([.5 ,0 ,0 ,0])):
        """
        generate the stick model of the nextage robot in panda3d
        snp means stick nodepath

        :param hrp5robot: the Hrp5Robot object, see Hrp5robot.py
        :param rgtrbga: color of right arm
        :param lftrgba: color of left arm
        :return: null

        author: weiwei
        date: 20170613
        """

        hrp5nstick = NodePath("hrp5nstick")
        i = 0
        while i != -1:
            sticknp = self.pggen.gendumbbell(spos=hrp5nrobot.rgtarm[i]['linkpos'],
                                             epos=hrp5nrobot.rgtarm[i]['linkend'],
                                             thickness=20, rgba=rgtrbga)
            i = hrp5nrobot.rgtarm[i]['child']
            sticknp.reparentTo(hrp5nstick)
        i = 0
        while i != -1:
            sticknp = self.pggen.gendumbbell(spos=hrp5nrobot.lftarm[i]['linkpos'],
                                             epos=hrp5nrobot.lftarm[i]['linkend'],
                                             thickness=20, rgba=lftrgba)
            i = hrp5nrobot.lftarm[i]['child']
            sticknp.reparentTo(hrp5nstick)

        return hrp5nstick

    def genmnp(self, hrp5nrobot, jawwidthrgt = None, jawwidthlft = None):
        """
        generate a panda3d nodepath for the hrp5robot
        mnp indicates this function generates a mesh nodepath

        :param hrp5robot: the Hrp5Robot object, see hrp5.py
        :return: a nodepath which is ready to be plotted using plotmesh

        author: weiwei
        date: 20170613
        """

        identmat4 = Mat4.identMat()
        # body
        self.__hrp5nbody_nodepath.setMat(identmat4)
        self.__hrp5nbody_nodepath.setZ(self.__hrp5nmnp, 0)
        # chest 0
        self.__hrp5nchest0_nodepath.setMat(identmat4)
        self.__hrp5nchest0_nodepath.setZ(self.__hrp5nmnp, 274)
        # chest 1
        self.__hrp5nchest1_nodepath.setMat(identmat4)
        self.__hrp5nchest1_nodepath.setColor(.7,.7,.2,1)
        # chest 2
        hrp5nchest2_rotmat = pg.cvtMat4(hrp5nrobot.base['rotmat'])
        self.__hrp5nchest2_nodepath.setMat(hrp5nchest2_rotmat)
        self.__hrp5nchest2_nodepath.setColor(.7,.7,.7,1)
        # head 0
        self.__hrp5nhead0_nodepath.setMat(identmat4)
        self.__hrp5nhead0_nodepath.setH(hrp5nrobot.initjnts[0])
        self.__hrp5nhead0_nodepath.setX(32)
        self.__hrp5nhead0_nodepath.setZ(521)
        # head 1
        self.__hrp5nhead1_nodepath.setP(hrp5nrobot.initjnts[1])
        self.__hrp5nhead1_nodepath.setX(40)
        self.__hrp5nhead1_nodepath.setColor(.5,.5,.5,1)

        # rgtarm 0
        hrp5nrgtarmlj0_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[1]['rotmat'], hrp5nrobot.rgtarm[1]['linkpos'])
        self.__hrp5nrgtarmlj0_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj0_rotmat)
        self.__hrp5nrgtarmlj0_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 1
        hrp5nrgtarmlj1_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[2]['rotmat'], hrp5nrobot.rgtarm[2]['linkpos'])
        self.__hrp5nrgtarmlj1_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj1_rotmat)
        self.__hrp5nrgtarmlj1_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 2
        hrp5nrgtarmlj2_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[3]['rotmat'], hrp5nrobot.rgtarm[3]['linkpos'])
        self.__hrp5nrgtarmlj2_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj2_rotmat)
        self.__hrp5nrgtarmlj2_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 3
        hrp5nrgtarmlj3_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[4]['rotmat'], hrp5nrobot.rgtarm[4]['linkpos'])
        self.__hrp5nrgtarmlj3_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj3_rotmat)
        self.__hrp5nrgtarmlj3_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 4
        hrp5nrgtarmlj4_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[5]['rotmat'], hrp5nrobot.rgtarm[5]['linkpos'])
        self.__hrp5nrgtarmlj4_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj4_rotmat)
        self.__hrp5nrgtarmlj4_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 5
        hrp5nrgtarmlj5_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[6]['rotmat'], hrp5nrobot.rgtarm[6]['linkpos'])
        self.__hrp5nrgtarmlj5_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj5_rotmat)
        self.__hrp5nrgtarmlj5_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 6
        hrp5nrgtarmlj6_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[7]['rotmat'], hrp5nrobot.rgtarm[7]['linkpos'])
        self.__hrp5nrgtarmlj6_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj6_rotmat)
        self.__hrp5nrgtarmlj6_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 7
        hrp5nrgtarmlj7_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[8]['rotmat'], hrp5nrobot.rgtarm[8]['linkpos'])
        self.__hrp5nrgtarmlj7_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj7_rotmat)
        self.__hrp5nrgtarmlj7_nodepath.setColor(.5, .5, .5, 1)

        # lftarm 0
        hrp5nlftarmlj0_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[1]['rotmat'], hrp5nrobot.lftarm[1]['linkpos'])
        self.__hrp5nlftarmlj0_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj0_rotmat)
        self.__hrp5nlftarmlj0_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 1
        hrp5nlftarmlj1_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[2]['rotmat'], hrp5nrobot.lftarm[2]['linkpos'])
        self.__hrp5nlftarmlj1_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj1_rotmat)
        self.__hrp5nlftarmlj1_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 2
        hrp5nlftarmlj2_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[3]['rotmat'], hrp5nrobot.lftarm[3]['linkpos'])
        self.__hrp5nlftarmlj2_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj2_rotmat)
        self.__hrp5nlftarmlj2_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 3
        hrp5nlftarmlj3_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[4]['rotmat'], hrp5nrobot.lftarm[4]['linkpos'])
        self.__hrp5nlftarmlj3_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj3_rotmat)
        self.__hrp5nlftarmlj3_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 4
        hrp5nlftarmlj4_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[5]['rotmat'], hrp5nrobot.lftarm[5]['linkpos'])
        self.__hrp5nlftarmlj4_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj4_rotmat)
        self.__hrp5nlftarmlj4_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 5
        hrp5nlftarmlj5_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[6]['rotmat'], hrp5nrobot.lftarm[6]['linkpos'])
        self.__hrp5nlftarmlj5_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj5_rotmat)
        self.__hrp5nlftarmlj5_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 6
        hrp5nlftarmlj6_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[7]['rotmat'], hrp5nrobot.lftarm[7]['linkpos'])
        self.__hrp5nlftarmlj6_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj6_rotmat)
        self.__hrp5nlftarmlj6_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 7
        hrp5nlftarmlj7_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[8]['rotmat'], hrp5nrobot.lftarm[8]['linkpos'])
        self.__hrp5nlftarmlj7_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj7_rotmat)
        self.__hrp5nlftarmlj7_nodepath.setColor(.5, .5, .5, 1)

        # rgthand
        hrp5nrobotrgtarmlj9_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[9]['rotmat'], hrp5nrobot.rgtarm[9]['linkpos'])
        self.hrp5nrobotrgthnd.setMat(self.__hrp5nmnp, hrp5nrobotrgtarmlj9_rotmat)
        if jawwidthrgt is not None:
            self.hrp5nrobotrgthnd.setJawwidth(jawwidthrgt)

        # lfthand
        hrp5nrobotlftarmlj9_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[9]['rotmat'], hrp5nrobot.lftarm[9]['linkpos'])
        self.hrp5nrobotlfthnd.setMat(self.__hrp5nmnp, hrp5nrobotlftarmlj9_rotmat)
        if jawwidthlft is not None:
            self.hrp5nrobotlfthnd.setJawwidth(jawwidthlft)

        return copy.deepcopy(self.__hrp5nmnp)

    def genmnp_list(self, hrp5nrobot, jawwidthrgt = None, jawwidthlft = None):
        """
        generate a panda3d nodepath for the hrp5robot
        mnp indicates this function generates a mesh nodepath

        return [[right arm mnp list], [leftarm mnp list], [body mnp list]]
        where one arm mnp list is ordered from base to end-effector
        this function is designed for collision detection

        :param hrp5robot: the Hrp5Robot object, see hrp5.py
        :return: a nodepath which is ready to be plotted using plotmesh

        author: weiwei
        date: 20170613
        """

        identmat4 = Mat4.identMat()
        # body
        self.__hrp5nbody_nodepath.setMat(identmat4)
        self.__hrp5nbody_nodepath.setZ(self.__hrp5nmnp, 0)
        # chest 0
        self.__hrp5nchest0_nodepath.setMat(identmat4)
        self.__hrp5nchest0_nodepath.setZ(self.__hrp5nmnp, 274)
        # chest 1
        self.__hrp5nchest1_nodepath.setMat(identmat4)
        self.__hrp5nchest1_nodepath.setColor(.7,.7,.2,1)
        # chest 2
        hrp5nchest2_rotmat = pg.cvtMat4(hrp5nrobot.base['rotmat'])
        self.__hrp5nchest2_nodepath.setMat(hrp5nchest2_rotmat)
        self.__hrp5nchest2_nodepath.setColor(.7,.7,.7,1)
        # head 0
        self.__hrp5nhead0_nodepath.setMat(identmat4)
        self.__hrp5nhead0_nodepath.setH(hrp5nrobot.initjnts[0])
        self.__hrp5nhead0_nodepath.setX(32)
        self.__hrp5nhead0_nodepath.setZ(521)
        # head 1
        self.__hrp5nhead1_nodepath.setP(hrp5nrobot.initjnts[1])
        self.__hrp5nhead1_nodepath.setX(40)
        self.__hrp5nhead1_nodepath.setColor(.5,.5,.5,1)

        # rgtarm 0
        hrp5nrgtarmlj0_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[1]['rotmat'], hrp5nrobot.rgtarm[1]['linkpos'])
        self.__hrp5nrgtarmlj0_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj0_rotmat)
        self.__hrp5nrgtarmlj0_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 1
        hrp5nrgtarmlj1_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[2]['rotmat'], hrp5nrobot.rgtarm[2]['linkpos'])
        self.__hrp5nrgtarmlj1_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj1_rotmat)
        self.__hrp5nrgtarmlj1_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 2
        hrp5nrgtarmlj2_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[3]['rotmat'], hrp5nrobot.rgtarm[3]['linkpos'])
        self.__hrp5nrgtarmlj2_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj2_rotmat)
        self.__hrp5nrgtarmlj2_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 3
        hrp5nrgtarmlj3_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[4]['rotmat'], hrp5nrobot.rgtarm[4]['linkpos'])
        self.__hrp5nrgtarmlj3_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj3_rotmat)
        self.__hrp5nrgtarmlj3_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 4
        hrp5nrgtarmlj4_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[5]['rotmat'], hrp5nrobot.rgtarm[5]['linkpos'])
        self.__hrp5nrgtarmlj4_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj4_rotmat)
        self.__hrp5nrgtarmlj4_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 5
        hrp5nrgtarmlj5_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[6]['rotmat'], hrp5nrobot.rgtarm[6]['linkpos'])
        self.__hrp5nrgtarmlj5_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj5_rotmat)
        self.__hrp5nrgtarmlj5_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 6
        hrp5nrgtarmlj6_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[7]['rotmat'], hrp5nrobot.rgtarm[7]['linkpos'])
        self.__hrp5nrgtarmlj6_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj6_rotmat)
        self.__hrp5nrgtarmlj6_nodepath.setColor(.5, .5, .5, 1)
        # rgtarm 7
        hrp5nrgtarmlj7_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[8]['rotmat'], hrp5nrobot.rgtarm[8]['linkpos'])
        self.__hrp5nrgtarmlj7_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj7_rotmat)
        self.__hrp5nrgtarmlj7_nodepath.setColor(.5, .5, .5, 1)

        # lftarm 0
        hrp5nlftarmlj0_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[1]['rotmat'], hrp5nrobot.lftarm[1]['linkpos'])
        self.__hrp5nlftarmlj0_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj0_rotmat)
        self.__hrp5nlftarmlj0_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 1
        hrp5nlftarmlj1_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[2]['rotmat'], hrp5nrobot.lftarm[2]['linkpos'])
        self.__hrp5nlftarmlj1_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj1_rotmat)
        self.__hrp5nlftarmlj1_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 2
        hrp5nlftarmlj2_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[3]['rotmat'], hrp5nrobot.lftarm[3]['linkpos'])
        self.__hrp5nlftarmlj2_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj2_rotmat)
        self.__hrp5nlftarmlj2_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 3
        hrp5nlftarmlj3_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[4]['rotmat'], hrp5nrobot.lftarm[4]['linkpos'])
        self.__hrp5nlftarmlj3_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj3_rotmat)
        self.__hrp5nlftarmlj3_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 4
        hrp5nlftarmlj4_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[5]['rotmat'], hrp5nrobot.lftarm[5]['linkpos'])
        self.__hrp5nlftarmlj4_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj4_rotmat)
        self.__hrp5nlftarmlj4_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 5
        hrp5nlftarmlj5_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[6]['rotmat'], hrp5nrobot.lftarm[6]['linkpos'])
        self.__hrp5nlftarmlj5_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj5_rotmat)
        self.__hrp5nlftarmlj5_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 6
        hrp5nlftarmlj6_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[7]['rotmat'], hrp5nrobot.lftarm[7]['linkpos'])
        self.__hrp5nlftarmlj6_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj6_rotmat)
        self.__hrp5nlftarmlj6_nodepath.setColor(.5, .5, .5, 1)
        # lftarm 7
        hrp5nlftarmlj7_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[8]['rotmat'], hrp5nrobot.lftarm[8]['linkpos'])
        self.__hrp5nlftarmlj7_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj7_rotmat)
        self.__hrp5nlftarmlj7_nodepath.setColor(.5, .5, .5, 1)

        # rgthand
        hrp5nrobotrgtarmlj9_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[9]['rotmat'], hrp5nrobot.rgtarm[9]['linkpos'])
        self.hrp5nrobotrgthnd.setMat(self.__hrp5nmnp, hrp5nrobotrgtarmlj9_rotmat)
        if jawwidthrgt is not None:
            self.hrp5nrobotrgthnd.setJawwidth(jawwidthrgt)

        # lfthand
        hrp5nrobotlftarmlj9_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[9]['rotmat'], hrp5nrobot.lftarm[9]['linkpos'])
        self.hrp5nrobotlfthnd.setMat(self.__hrp5nmnp, hrp5nrobotlftarmlj9_rotmat)
        if jawwidthlft is not None:
            self.hrp5nrobotlfthnd.setJawwidth(jawwidthlft)

        return copy.deepcopy([[self.__hrp5nrgtarmlj0_nodepath, self.__hrp5nrgtarmlj1_nodepath, self.__hrp5nrgtarmlj2_nodepath,
                 self.__hrp5nrgtarmlj3_nodepath, self.__hrp5nrgtarmlj4_nodepath, self.__hrp5nrgtarmlj5_nodepath,
                 self.__hrp5nrgtarmlj6_nodepath, self.__hrp5nrgtarmlj7_nodepath, self.hrp5nrobotrgthnd.handnp],
                [self.__hrp5nlftarmlj0_nodepath, self.__hrp5nlftarmlj1_nodepath, self.__hrp5nlftarmlj2_nodepath,
                 self.__hrp5nlftarmlj3_nodepath, self.__hrp5nlftarmlj4_nodepath, self.__hrp5nlftarmlj5_nodepath,
                 self.__hrp5nlftarmlj6_nodepath, self.__hrp5nlftarmlj7_nodepath, self.hrp5nrobotlfthnd.handnp],
                [self.__hrp5nbody_nodepath]])

    def genmnp_nm(self,hrp5nrobot, jawwidthrgt = None, jawwidthlft = None, plotcolor=[.5,.5,.5,.3]):
        """
        generate a panda3d nodepath for the hrp5robot
        mnp indicates this function generates a mesh nodepath

        nm indicates the robots include no material
        since I didn't remove the materials on the hrp5 model,
        this function is essentially the same as genmnp_nm

        :param hrp5robot: the Hrp5Robot object, see hrp5.py
        :return: a nodepath which is ready to be plotted using plotmesh

        author: weiwei
        date: 20170613
        """

        identmat4 = Mat4.identMat()
        # body
        self.__hrp5nbody_nodepath.setMat(identmat4)
        self.__hrp5nbody_nodepath.setZ(self.__hrp5nmnp, 0)
        # chest 0
        self.__hrp5nchest0_nodepath.setMat(identmat4)
        self.__hrp5nchest0_nodepath.setZ(self.__hrp5nmnp, 274)
        # chest 1
        self.__hrp5nchest1_nodepath.setMat(identmat4)
        self.__hrp5nchest1_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # chest 2
        hrp5nchest2_rotmat = pg.cvtMat4(hrp5nrobot.base['rotmat'])
        self.__hrp5nchest2_nodepath.setMat(hrp5nchest2_rotmat)
        self.__hrp5nchest2_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # head 0
        self.__hrp5nhead0_nodepath.setMat(identmat4)
        self.__hrp5nhead0_nodepath.setH(hrp5nrobot.initjnts[0])
        self.__hrp5nhead0_nodepath.setX(32)
        self.__hrp5nhead0_nodepath.setZ(521)
        # head 1
        self.__hrp5nhead1_nodepath.setP(hrp5nrobot.initjnts[1])
        self.__hrp5nhead1_nodepath.setX(40)
        self.__hrp5nhead1_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])

        # rgtarm 0
        hrp5nrgtarmlj0_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[1]['rotmat'], hrp5nrobot.rgtarm[1]['linkpos'])
        self.__hrp5nrgtarmlj0_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj0_rotmat)
        self.__hrp5nrgtarmlj0_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # rgtarm 1
        hrp5nrgtarmlj1_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[2]['rotmat'], hrp5nrobot.rgtarm[2]['linkpos'])
        self.__hrp5nrgtarmlj1_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj1_rotmat)
        self.__hrp5nrgtarmlj1_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # rgtarm 2
        hrp5nrgtarmlj2_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[3]['rotmat'], hrp5nrobot.rgtarm[3]['linkpos'])
        self.__hrp5nrgtarmlj2_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj2_rotmat)
        self.__hrp5nrgtarmlj2_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # rgtarm 3
        hrp5nrgtarmlj3_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[4]['rotmat'], hrp5nrobot.rgtarm[4]['linkpos'])
        self.__hrp5nrgtarmlj3_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj3_rotmat)
        self.__hrp5nrgtarmlj3_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # rgtarm 4
        hrp5nrgtarmlj4_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[5]['rotmat'], hrp5nrobot.rgtarm[5]['linkpos'])
        self.__hrp5nrgtarmlj4_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj4_rotmat)
        self.__hrp5nrgtarmlj4_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # rgtarm 5
        hrp5nrgtarmlj5_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[6]['rotmat'], hrp5nrobot.rgtarm[6]['linkpos'])
        self.__hrp5nrgtarmlj5_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj5_rotmat)
        self.__hrp5nrgtarmlj5_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # rgtarm 6
        hrp5nrgtarmlj6_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[7]['rotmat'], hrp5nrobot.rgtarm[7]['linkpos'])
        self.__hrp5nrgtarmlj6_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj6_rotmat)
        self.__hrp5nrgtarmlj6_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # rgtarm 7
        hrp5nrgtarmlj7_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[8]['rotmat'], hrp5nrobot.rgtarm[8]['linkpos'])
        self.__hrp5nrgtarmlj7_nodepath.setMat(self.__hrp5nmnp, hrp5nrgtarmlj7_rotmat)
        self.__hrp5nrgtarmlj7_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])

        # lftarm 0
        hrp5nlftarmlj0_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[1]['rotmat'], hrp5nrobot.lftarm[1]['linkpos'])
        self.__hrp5nlftarmlj0_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj0_rotmat)
        self.__hrp5nlftarmlj0_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # lftarm 1
        hrp5nlftarmlj1_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[2]['rotmat'], hrp5nrobot.lftarm[2]['linkpos'])
        self.__hrp5nlftarmlj1_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj1_rotmat)
        self.__hrp5nlftarmlj1_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # lftarm 2
        hrp5nlftarmlj2_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[3]['rotmat'], hrp5nrobot.lftarm[3]['linkpos'])
        self.__hrp5nlftarmlj2_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj2_rotmat)
        self.__hrp5nlftarmlj2_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # lftarm 3
        hrp5nlftarmlj3_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[4]['rotmat'], hrp5nrobot.lftarm[4]['linkpos'])
        self.__hrp5nlftarmlj3_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj3_rotmat)
        self.__hrp5nlftarmlj3_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # lftarm 4
        hrp5nlftarmlj4_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[5]['rotmat'], hrp5nrobot.lftarm[5]['linkpos'])
        self.__hrp5nlftarmlj4_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj4_rotmat)
        self.__hrp5nlftarmlj4_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # lftarm 5
        hrp5nlftarmlj5_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[6]['rotmat'], hrp5nrobot.lftarm[6]['linkpos'])
        self.__hrp5nlftarmlj5_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj5_rotmat)
        self.__hrp5nlftarmlj5_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # lftarm 6
        hrp5nlftarmlj6_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[7]['rotmat'], hrp5nrobot.lftarm[7]['linkpos'])
        self.__hrp5nlftarmlj6_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj6_rotmat)
        self.__hrp5nlftarmlj6_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])
        # lftarm 7
        hrp5nlftarmlj7_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[8]['rotmat'], hrp5nrobot.lftarm[8]['linkpos'])
        self.__hrp5nlftarmlj7_nodepath.setMat(self.__hrp5nmnp, hrp5nlftarmlj7_rotmat)
        self.__hrp5nlftarmlj7_nodepath.setColor(plotcolor[0], plotcolor[1], plotcolor[2], plotcolor[3])

        # rgthand
        hrp5nrobotrgtarmlj9_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[9]['rotmat'], hrp5nrobot.rgtarm[9]['linkpos'])
        self.hrp5nrobotrgthnd.setMat(self.__hrp5nmnp, hrp5nrobotrgtarmlj9_rotmat)
        if jawwidthrgt is not None:
            self.hrp5nrobotrgthnd.setJawwidth(jawwidthrgt)

        # lfthand
        hrp5nrobotlftarmlj9_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[9]['rotmat'], hrp5nrobot.lftarm[9]['linkpos'])
        self.hrp5nrobotlfthnd.setMat(self.__hrp5nmnp, hrp5nrobotlftarmlj9_rotmat)
        if jawwidthlft is not None:
            self.hrp5nrobotlfthnd.setJawwidth(jawwidthlft)

        return copy.deepcopy(self.__hrp5nmnp)
