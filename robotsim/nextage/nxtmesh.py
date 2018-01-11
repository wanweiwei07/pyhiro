import os

import numpy as np
from manipulation.grip.robotiq85 import rtq85
from manipulation.grip.hrp5three import hrp5three
from panda3d.core import *
import copy

import pandaplotutils.pandageom as pg

class NxtMesh(object):
    """
    generate nxtmesh

    NOTE: it is unnecessary to attach a mnp to render repeatedly
    once attached, it is always there. update the joint angles will change the attached model directly
    """

    def __init__(self, handpkg):
        """
        load models

        :param handpkg:
        author: weiwei
        date: 20180109
        """

        self.__nxtmnp = NodePath("nxtmesh")

        ##########
        ### load the model files of the robots
        ##########
        this_dir, this_filename = os.path.split(__file__)

        nxtwaist_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_waist.egg"))
        nxtbody_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_body.egg"))
        nxthead_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_head.egg"))

        self.__nxtwaist_nodepath = NodePath("nxtwaist")
        self.__nxtbody_nodepath = NodePath("nxtbody")
        self.__nxthead_nodepath = NodePath("nxthead")

        nxtwaist_model = loader.loadModel(nxtwaist_filepath)
        nxtbody_model = loader.loadModel(nxtbody_filepath)
        nxthead_model = loader.loadModel(nxthead_filepath)

        nxtwaist_model.instanceTo(self.__nxtwaist_nodepath)
        nxtbody_model.instanceTo(self.__nxtbody_nodepath)
        nxthead_model.instanceTo(self.__nxthead_nodepath)

        self.__nxtwaist_nodepath.reparentTo(self.__nxtmnp)
        self.__nxtbody_nodepath.reparentTo(self.__nxtwaist_nodepath)
        self.__nxthead_nodepath.setZ(569.5)
        self.__nxthead_nodepath.reparentTo(self.__nxtwaist_nodepath)

        # rgtarm
        nxtrgtarmlj0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj0.egg"))
        nxtrgtarmlj0_model = loader.loadModel(nxtrgtarmlj0_filepath)
        self.__nxtrgtarmlj0_nodepath = NodePath("nxtrgtarmlj0_nodepath")
        nxtrgtarmlj0_model.instanceTo(self.__nxtrgtarmlj0_nodepath)
        self.__nxtrgtarmlj0_nodepath.reparentTo(self.__nxtmnp)

        nxtrgtarmlj1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj1.egg"))
        nxtrgtarmlj1_model = loader.loadModel(nxtrgtarmlj1_filepath)
        self.__nxtrgtarmlj1_nodepath = NodePath("nxtrgtarmlj1_nodepath")
        nxtrgtarmlj1_model.instanceTo(self.__nxtrgtarmlj1_nodepath)
        self.__nxtrgtarmlj1_nodepath.reparentTo(self.__nxtmnp)

        nxtrgtarmlj2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj2.egg"))
        nxtrgtarmlj2_model = loader.loadModel(nxtrgtarmlj2_filepath)
        self.__nxtrgtarmlj2_nodepath = NodePath("nxtrgtarmlj2_nodepath")
        nxtrgtarmlj2_model.instanceTo(self.__nxtrgtarmlj2_nodepath)
        self.__nxtrgtarmlj2_nodepath.reparentTo(self.__nxtmnp)

        nxtrgtarmlj3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj3.egg"))
        nxtrgtarmlj3_model = loader.loadModel(nxtrgtarmlj3_filepath)
        self.__nxtrgtarmlj3_nodepath = NodePath("nxtrgtarmlj3_nodepath")
        nxtrgtarmlj3_model.instanceTo(self.__nxtrgtarmlj3_nodepath)
        self.__nxtrgtarmlj3_nodepath.reparentTo(self.__nxtmnp)

        nxtrgtarmlj4_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_rgtarm_lj4.egg"))
        nxtrgtarmlj4_model = loader.loadModel(nxtrgtarmlj4_filepath)
        self.__nxtrgtarmlj4_nodepath = NodePath("nxtrgtarmlj4_nodepath")
        nxtrgtarmlj4_model.instanceTo(self.__nxtrgtarmlj4_nodepath)
        self.__nxtrgtarmlj4_nodepath.reparentTo(self.__nxtmnp)

        # lftarm
        nxtlftarmlj0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj0.egg"))
        nxtlftarmlj0_model = loader.loadModel(nxtlftarmlj0_filepath)
        self.__nxtlftarmlj0_nodepath = NodePath("nxtlftarmlj0_nodepath")
        nxtlftarmlj0_model.instanceTo(self.__nxtlftarmlj0_nodepath)
        self.__nxtlftarmlj0_nodepath.reparentTo(self.__nxtmnp)

        nxtlftarmlj1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj1.egg"))
        nxtlftarmlj1_model = loader.loadModel(nxtlftarmlj1_filepath)
        self.__nxtlftarmlj1_nodepath = NodePath("nxtlftarmlj1_nodepath")
        nxtlftarmlj1_model.instanceTo(self.__nxtlftarmlj1_nodepath)
        self.__nxtlftarmlj1_nodepath.reparentTo(self.__nxtmnp)

        nxtlftarmlj2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj2.egg"))
        nxtlftarmlj2_model = loader.loadModel(nxtlftarmlj2_filepath)
        self.__nxtlftarmlj2_nodepath = NodePath("nxtlftarmlj2_nodepath")
        nxtlftarmlj2_model.instanceTo(self.__nxtlftarmlj2_nodepath)
        self.__nxtlftarmlj2_nodepath.reparentTo(self.__nxtmnp)

        nxtlftarmlj3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj3.egg"))
        nxtlftarmlj3_model = loader.loadModel(nxtlftarmlj3_filepath)
        self.__nxtlftarmlj3_nodepath = NodePath("nxtlftarmlj3_nodepath")
        nxtlftarmlj3_model.instanceTo(self.__nxtlftarmlj3_nodepath)
        self.__nxtlftarmlj3_nodepath.reparentTo(self.__nxtmnp)

        nxtlftarmlj4_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj4.egg"))
        nxtlftarmlj4_model = loader.loadModel(nxtlftarmlj4_filepath)
        self.__nxtlftarmlj4_nodepath = NodePath("nxtlftarmlj4_nodepath")
        nxtlftarmlj4_model.instanceTo(self.__nxtlftarmlj4_nodepath)
        self.__nxtlftarmlj4_nodepath.reparentTo(self.__nxtmnp)

        # rgthnd
        self.nxtrgthnd = handpkg.newHand('rgt')
        self.nxtrgthnd.reparentTo(self.__nxtmnp)

        # lfthnd
        self.nxtlfthnd = handpkg.newHand('lft')
        self.nxtlfthnd.reparentTo(self.__nxtmnp)

        ##########
        ### load the model files of sticks
        ##########

        self.pggen = pg.PandaGeomGen()

    def gensnp(self, nxtrobot, rgtrbga=np.array([.5 ,0 ,0 ,0]), lftrgba=np.array([.5 ,0 ,0 ,0])):
        """
        generate the stick model of the nextage robot in panda3d
        snp means stick nodepath

        :param nxtrobot: the nxtrobot object, see Hrp5robot.py
        :param rgtrbga: color of right arm
        :param lftrgba: color of left arm
        :return: null

        author: weiwei
        date: 20180109
        """

        nxtstick = NodePath("nxtstick")
        i = 0
        while i != -1:
            sticknp = self.pggen.gendumbbell(spos=nxtrobot.rgtarm[i]['linkpos'],
                                             epos=nxtrobot.rgtarm[i]['linkend'],
                                             thickness=20, rgba=rgtrbga)
            i = nxtrobot.rgtarm[i]['child']
            sticknp.reparentTo(nxtstick)
        i = 0
        while i != -1:
            sticknp = self.pggen.gendumbbell(spos=nxtrobot.lftarm[i]['linkpos'],
                                             epos=nxtrobot.lftarm[i]['linkend'],
                                             thickness=20, rgba=lftrgba)
            i = nxtrobot.lftarm[i]['child']
            sticknp.reparentTo(nxtstick)

        return nxtstick

    def genmnp(self, nxtrobot, jawwidthrgt = None, jawwidthlft = None):
        """
        generate a panda3d nodepath for the nxtrobot
        mnp indicates this function generates a mesh nodepath

        :param nxtrobot: the nxtrobot object, see nxtrobot.py
        :return: a nodepath which is ready to be plotted using plotmesh

        author: weiwei
        date: 20180109
        """

        identmat4 = Mat4.identMat()
        # body

        nxtwaist_rotmat = pg.cvtMat4(nxtrobot.base['rotmat'])
        self.__nxtwaist_nodepath.setMat(nxtwaist_rotmat)

        # rgtarm
        nxtrgtarmlj0_rotmat = pg.cvtMat4(nxtrobot.rgtarm[1]['rotmat'], nxtrobot.rgtarm[1]['linkpos'])
        self.__nxtrgtarmlj0_nodepath.setMat(nxtrgtarmlj0_rotmat)
        nxtrgtarmlj1_rotmat = pg.cvtMat4(nxtrobot.rgtarm[2]['rotmat'], nxtrobot.rgtarm[2]['linkpos'])
        self.__nxtrgtarmlj1_nodepath.setMat(nxtrgtarmlj1_rotmat)
        nxtrgtarmlj2_rotmat = pg.cvtMat4(nxtrobot.rgtarm[3]['rotmat'], nxtrobot.rgtarm[3]['linkpos'])
        self.__nxtrgtarmlj2_nodepath.setMat(nxtrgtarmlj2_rotmat)
        nxtrgtarmlj3_rotmat = pg.cvtMat4(nxtrobot.rgtarm[4]['rotmat'], nxtrobot.rgtarm[4]['linkpos'])
        self.__nxtrgtarmlj3_nodepath.setMat(nxtrgtarmlj3_rotmat)
        nxtrgtarmlj4_rotmat = pg.cvtMat4(nxtrobot.rgtarm[5]['rotmat'], nxtrobot.rgtarm[5]['linkpos'])
        self.__nxtrgtarmlj4_nodepath.setMat(nxtrgtarmlj4_rotmat)

        # lftarm
        nxtlftarmlj0_rotmat = pg.cvtMat4(nxtrobot.lftarm[1]['rotmat'], nxtrobot.lftarm[1]['linkpos'])
        self.__nxtlftarmlj0_nodepath.setMat(nxtlftarmlj0_rotmat)
        nxtlftarmlj1_rotmat = pg.cvtMat4(nxtrobot.lftarm[2]['rotmat'], nxtrobot.lftarm[2]['linkpos'])
        self.__nxtlftarmlj1_nodepath.setMat(nxtlftarmlj1_rotmat)
        nxtlftarmlj2_rotmat = pg.cvtMat4(nxtrobot.lftarm[3]['rotmat'], nxtrobot.lftarm[3]['linkpos'])
        self.__nxtlftarmlj2_nodepath.setMat(nxtlftarmlj2_rotmat)
        nxtlftarmlj3_rotmat = pg.cvtMat4(nxtrobot.lftarm[4]['rotmat'], nxtrobot.lftarm[4]['linkpos'])
        self.__nxtlftarmlj3_nodepath.setMat(nxtlftarmlj3_rotmat)
        nxtlftarmlj4_rotmat = pg.cvtMat4(nxtrobot.lftarm[5]['rotmat'], nxtrobot.lftarm[5]['linkpos'])
        self.__nxtlftarmlj4_nodepath.setMat(nxtlftarmlj4_rotmat)

        # rgthand
        nxtlftarmlj5_rotmat = pg.cvtMat4(nxtrobot.rgtarm[6]['rotmat'], nxtrobot.rgtarm[6]['linkpos'])
        self.nxtrgthnd.setMat(nxtlftarmlj5_rotmat)
        pg.plotAxisSelf(self.__nxtmnp, nxtrobot.rgtarm[6]['linkend'], nxtlftarmlj5_rotmat)
        if jawwidthrgt is not None:
            self.nxtrgthnd.setJawwidth(jawwidthrgt)

        # lfthand
        nxtlftarmlj5_rotmat = pg.cvtMat4(nxtrobot.lftarm[6]['rotmat'], nxtrobot.lftarm[6]['linkpos'])
        self.nxtlfthnd.setMat(nxtlftarmlj5_rotmat)
        pg.plotAxisSelf(self.__nxtmnp, nxtrobot.lftarm[6]['linkend'], nxtlftarmlj5_rotmat)
        if jawwidthlft is not None:
            self.nxtlfthnd.setJawwidth(jawwidthlft)

        return copy.deepcopy(self.__nxtmnp)
