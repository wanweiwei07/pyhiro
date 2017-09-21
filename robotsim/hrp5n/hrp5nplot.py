import os

import numpy as np
from manipulation.grip.robotiq85 import rtq85
from manipulation.grip.hrp5three import hrp5three
from panda3d.core import *

import pandaplotutils.pandageom as pg

# TODO, scale up the models for collision detection

def plotstick(pandanp, hrp5robot, rgtrbga=np.array([.5 ,0 ,0 ,0]), lftrgba=np.array([.5 ,0 ,0 ,0])):
    """
    plot the stick model of the nextage robot in panda3d

    :param pandanp: a panda3d nodepath
    :param hrp5robot: the Hrp5Robot object, see Hrp5robot.py
    :param rgtrbga: color of right arm
    :param lftrgba: color of left arm
    :return: null

    author: weiwei
    date: 20161202
    """

    i = 0
    while i != -1:
        pg.plotDumbbell(pandanp, spos=hrp5robot.rgtarm[i]['linkpos'], epos=hrp5robot.rgtarm[i]['linkend'],
                               thickness=20, rgba=rgtrbga)
        i = hrp5robot.rgtarm[i]['child']
    i = 0
    while i != -1:
        pg.plotDumbbell(pandanp, spos=hrp5robot.lftarm[i]['linkpos'], epos=hrp5robot.lftarm[i]['linkend'],
                               thickness=20, rgba=lftrgba)
        i = hrp5robot.lftarm[i]['child']

def genmnp(hrp5nrobot, handpkg, jawwidthrgt = None, jawwidthlft = None):
    """
    generate a panda3d nodepath for the hrp5robot
    mnp indicates this function generates a mesh nodepath

    :param hrp5robot: the Hrp5Robot object, see hrp5.py
    :return: a nodepath which is ready to be plotted using plotmesh

    author: weiwei
    date: 20161202
    """

    hrp5nmnp = NodePath("hrp5nmnp")

    this_dir, this_filename = os.path.split(__file__)

    # chest0-2, head1 (neck is not plotted)
    hrp5nbody_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Body.egg"))
    hrp5nchest0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Chest_Link0.egg"))
    hrp5nchest1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Chest_Link1.egg"))
    hrp5nchest2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Chest_Link2.egg"))
    hrp5nhead0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Head_Link0.egg"))
    hrp5nhead1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Head_Link1.egg"))

    hrp5nbody_model  = loader.loadModel(hrp5nbody_filepath)
    hrp5nchest0_model  = loader.loadModel(hrp5nchest0_filepath)
    hrp5nchest1_model = loader.loadModel(hrp5nchest1_filepath)
    hrp5nchest2_model = loader.loadModel(hrp5nchest2_filepath)
    hrp5nhead0_model = loader.loadModel(hrp5nhead0_filepath)
    hrp5nhead1_model = loader.loadModel(hrp5nhead1_filepath)

    hrp5nbody_nodepath = NodePath("hrp5nbody")
    hrp5nchest0_nodepath = NodePath("hrp5nchest0")
    hrp5nchest1_nodepath = NodePath("hrp5nchest1")
    hrp5nchest2_nodepath = NodePath("hrp5nchest2")
    hrp5nhead0_nodepath = NodePath("hrp5nhead0")
    hrp5nhead1_nodepath = NodePath("hrp5nhead1")

    # body
    hrp5nbody_model.instanceTo(hrp5nbody_nodepath)
    hrp5nbody_rotmat = Mat4.identMat()
    hrp5nbody_nodepath.setMat(hrp5nbody_rotmat)
    hrp5nbody_nodepath.setZ(0)
    # chest
    hrp5nchest0_model.instanceTo(hrp5nchest0_nodepath)
    hrp5nchest0_rotmat = Mat4.identMat()
    hrp5nchest0_nodepath.setMat(hrp5nchest0_rotmat)
    hrp5nchest0_nodepath.setZ(274)
    hrp5nchest1_model.instanceTo(hrp5nchest1_nodepath)
    hrp5nchest1_nodepath.setColor(.7,.7,.2,1)
    hrp5nchest2_model.instanceTo(hrp5nchest2_nodepath)
    hrp5nchest2_rotmat = pg.cvtMat4(hrp5nrobot.base['rotmat'])
    hrp5nchest2_nodepath.setMat(hrp5nchest2_rotmat)
    hrp5nchest2_nodepath.setColor(.7,.7,.7,1)
    hrp5nhead0_model.instanceTo(hrp5nhead0_nodepath)
    hrp5nhead0_nodepath.setH(hrp5nrobot.initjnts[0])
    hrp5nhead0_nodepath.setX(32)
    hrp5nhead0_nodepath.setZ(521)
    hrp5nhead1_model.instanceTo(hrp5nhead1_nodepath)
    hrp5nhead1_nodepath.setP(hrp5nrobot.initjnts[1])
    hrp5nhead1_nodepath.setX(40)
    hrp5nhead1_nodepath.setColor(.5,.5,.5,1)

    hrp5nbody_nodepath.reparentTo(hrp5nmnp)
    hrp5nchest0_nodepath.reparentTo(hrp5nbody_nodepath)
    hrp5nchest1_nodepath.reparentTo(hrp5nchest0_nodepath)
    hrp5nchest2_nodepath.reparentTo(hrp5nchest1_nodepath)
    hrp5nhead0_nodepath.reparentTo(hrp5nchest2_nodepath)
    hrp5nhead1_nodepath.reparentTo(hrp5nhead0_nodepath)

    # rgtarm
    hrp5nrgtarmlj0_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rscap_Link0.egg"))
    hrp5nrgtarmlj0_model = loader.loadModel(hrp5nrgtarmlj0_filepath)
    hrp5nrgtarmlj0_nodepath = NodePath("nxtrgtarmlj0_nodepath")
    hrp5nrgtarmlj0_model.instanceTo(hrp5nrgtarmlj0_nodepath)
    hrp5nrgtarmlj0_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[1]['rotmat'], hrp5nrobot.rgtarm[1]['linkpos'])
    hrp5nrgtarmlj0_nodepath.setMat(hrp5nrgtarmlj0_rotmat)
    hrp5nrgtarmlj0_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj0_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj1_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link0.egg"))
    hrp5nrgtarmlj1_model = loader.loadModel(hrp5nrgtarmlj1_filepath)
    hrp5nrgtarmlj1_nodepath = NodePath("nxtrgtarmlj1_nodepath")
    hrp5nrgtarmlj1_model.instanceTo(hrp5nrgtarmlj1_nodepath)
    hrp5nrgtarmlj1_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[2]['rotmat'], hrp5nrobot.rgtarm[2]['linkpos'])
    hrp5nrgtarmlj1_nodepath.setMat(hrp5nrgtarmlj1_rotmat)
    hrp5nrgtarmlj1_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj1_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj2_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link1.egg"))
    hrp5nrgtarmlj2_model = loader.loadModel(hrp5nrgtarmlj2_filepath)
    hrp5nrgtarmlj2_nodepath = NodePath("nxtrgtarmlj2_nodepath")
    hrp5nrgtarmlj2_model.instanceTo(hrp5nrgtarmlj2_nodepath)
    hrp5nrgtarmlj2_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[3]['rotmat'], hrp5nrobot.rgtarm[3]['linkpos'])
    hrp5nrgtarmlj2_nodepath.setMat(hrp5nrgtarmlj2_rotmat)
    hrp5nrgtarmlj2_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj2_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj3_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link2.egg"))
    hrp5nrgtarmlj3_model = loader.loadModel(hrp5nrgtarmlj3_filepath)
    hrp5nrgtarmlj3_nodepath = NodePath("nxtrgtarmlj3_nodepath")
    hrp5nrgtarmlj3_model.instanceTo(hrp5nrgtarmlj3_nodepath)
    hrp5nrgtarmlj3_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[4]['rotmat'], hrp5nrobot.rgtarm[4]['linkpos'])
    hrp5nrgtarmlj3_nodepath.setMat(hrp5nrgtarmlj3_rotmat)
    hrp5nrgtarmlj3_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj3_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj4_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link3.egg"))
    hrp5nrgtarmlj4_model = loader.loadModel(hrp5nrgtarmlj4_filepath)
    hrp5nrgtarmlj4_nodepath = NodePath("nxtrgtarmlj4_nodepath")
    hrp5nrgtarmlj4_model.instanceTo(hrp5nrgtarmlj4_nodepath)
    hrp5nrgtarmlj4_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[5]['rotmat'], hrp5nrobot.rgtarm[5]['linkpos'])
    hrp5nrgtarmlj4_nodepath.setMat(hrp5nrgtarmlj4_rotmat)
    hrp5nrgtarmlj4_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj4_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj5_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link4.egg"))
    hrp5nrgtarmlj5_model = loader.loadModel(hrp5nrgtarmlj5_filepath)
    hrp5nrgtarmlj5_nodepath = NodePath("nxtrgtarmlj5_nodepath")
    hrp5nrgtarmlj5_model.instanceTo(hrp5nrgtarmlj5_nodepath)
    hrp5nrgtarmlj5_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[6]['rotmat'], hrp5nrobot.rgtarm[6]['linkpos'])
    hrp5nrgtarmlj5_nodepath.setMat(hrp5nrgtarmlj5_rotmat)
    hrp5nrgtarmlj5_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj5_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj6_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link5.egg"))
    hrp5nrgtarmlj6_model = loader.loadModel(hrp5nrgtarmlj6_filepath)
    hrp5nrgtarmlj6_nodepath = NodePath("nxtrgtarmlj6_nodepath")
    hrp5nrgtarmlj6_model.instanceTo(hrp5nrgtarmlj6_nodepath)
    hrp5nrgtarmlj6_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[7]['rotmat'], hrp5nrobot.rgtarm[7]['linkpos'])
    hrp5nrgtarmlj6_nodepath.setMat(hrp5nrgtarmlj6_rotmat)
    hrp5nrgtarmlj6_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj6_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj7_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link6.egg"))
    hrp5nrgtarmlj7_model = loader.loadModel(hrp5nrgtarmlj7_filepath)
    hrp5nrgtarmlj7_nodepath = NodePath("nxtrgtarmlj7_nodepath")
    hrp5nrgtarmlj7_model.instanceTo(hrp5nrgtarmlj7_nodepath)
    hrp5nrgtarmlj7_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[8]['rotmat'], hrp5nrobot.rgtarm[8]['linkpos'])
    hrp5nrgtarmlj7_nodepath.setMat(hrp5nrgtarmlj7_rotmat)
    hrp5nrgtarmlj7_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj7_nodepath.reparentTo(hrp5nmnp)

    # lftarm
    hrp5nlftarmlj0_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Lscap_Link0.egg"))
    hrp5nlftarmlj0_model = loader.loadModel(hrp5nlftarmlj0_filepath)
    hrp5nlftarmlj0_nodepath = NodePath("nxtlftarmlj0_nodepath")
    hrp5nlftarmlj0_model.instanceTo(hrp5nlftarmlj0_nodepath)
    hrp5nlftarmlj0_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[1]['rotmat'], hrp5nrobot.lftarm[1]['linkpos'])
    hrp5nlftarmlj0_nodepath.setMat(hrp5nlftarmlj0_rotmat)
    hrp5nlftarmlj0_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj0_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj1_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link0.egg"))
    hrp5nlftarmlj1_model = loader.loadModel(hrp5nlftarmlj1_filepath)
    hrp5nlftarmlj1_nodepath = NodePath("nxtlftarmlj1_nodepath")
    hrp5nlftarmlj1_model.instanceTo(hrp5nlftarmlj1_nodepath)
    hrp5nlftarmlj1_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[2]['rotmat'], hrp5nrobot.lftarm[2]['linkpos'])
    hrp5nlftarmlj1_nodepath.setMat(hrp5nlftarmlj1_rotmat)
    hrp5nlftarmlj1_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj1_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj2_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link1.egg"))
    hrp5nlftarmlj2_model = loader.loadModel(hrp5nlftarmlj2_filepath)
    hrp5nlftarmlj2_nodepath = NodePath("nxtlftarmlj2_nodepath")
    hrp5nlftarmlj2_model.instanceTo(hrp5nlftarmlj2_nodepath)
    hrp5nlftarmlj2_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[3]['rotmat'], hrp5nrobot.lftarm[3]['linkpos'])
    hrp5nlftarmlj2_nodepath.setMat(hrp5nlftarmlj2_rotmat)
    hrp5nlftarmlj2_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj2_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj3_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link2.egg"))
    hrp5nlftarmlj3_model = loader.loadModel(hrp5nlftarmlj3_filepath)
    hrp5nlftarmlj3_nodepath = NodePath("nxtlftarmlj3_nodepath")
    hrp5nlftarmlj3_model.instanceTo(hrp5nlftarmlj3_nodepath)
    hrp5nlftarmlj3_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[4]['rotmat'], hrp5nrobot.lftarm[4]['linkpos'])
    hrp5nlftarmlj3_nodepath.setMat(hrp5nlftarmlj3_rotmat)
    hrp5nlftarmlj3_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj3_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj4_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link3.egg"))
    hrp5nlftarmlj4_model = loader.loadModel(hrp5nlftarmlj4_filepath)
    hrp5nlftarmlj4_nodepath = NodePath("nxtlftarmlj4_nodepath")
    hrp5nlftarmlj4_model.instanceTo(hrp5nlftarmlj4_nodepath)
    hrp5nlftarmlj4_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[5]['rotmat'], hrp5nrobot.lftarm[5]['linkpos'])
    hrp5nlftarmlj4_nodepath.setMat(hrp5nlftarmlj4_rotmat)
    hrp5nlftarmlj4_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj4_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj5_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link4.egg"))
    hrp5nlftarmlj5_model = loader.loadModel(hrp5nlftarmlj5_filepath)
    hrp5nlftarmlj5_nodepath = NodePath("nxtlftarmlj5_nodepath")
    hrp5nlftarmlj5_model.instanceTo(hrp5nlftarmlj5_nodepath)
    hrp5nlftarmlj5_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[6]['rotmat'], hrp5nrobot.lftarm[6]['linkpos'])
    hrp5nlftarmlj5_nodepath.setMat(hrp5nlftarmlj5_rotmat)
    hrp5nlftarmlj5_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj5_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj6_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link5.egg"))
    hrp5nlftarmlj6_model = loader.loadModel(hrp5nlftarmlj6_filepath)
    hrp5nlftarmlj6_nodepath = NodePath("nxtlftarmlj6_nodepath")
    hrp5nlftarmlj6_model.instanceTo(hrp5nlftarmlj6_nodepath)
    hrp5nlftarmlj6_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[7]['rotmat'], hrp5nrobot.lftarm[7]['linkpos'])
    hrp5nlftarmlj6_nodepath.setMat(hrp5nlftarmlj6_rotmat)
    hrp5nlftarmlj6_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj6_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj7_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link6.egg"))
    hrp5nlftarmlj7_model = loader.loadModel(hrp5nlftarmlj7_filepath)
    hrp5nlftarmlj7_nodepath = NodePath("nxtlftarmlj7_nodepath")
    hrp5nlftarmlj7_model.instanceTo(hrp5nlftarmlj7_nodepath)
    hrp5nlftarmlj7_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[8]['rotmat'], hrp5nrobot.lftarm[8]['linkpos'])
    hrp5nlftarmlj7_nodepath.setMat(hrp5nlftarmlj7_rotmat)
    hrp5nlftarmlj7_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj7_nodepath.reparentTo(hrp5nmnp)

    # rgthnd
    hrp5nrobotrgthnd = handpkg.newHand('rgt')
    hrp5nrobotrgtarmlj9_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[9]['rotmat'], hrp5nrobot.rgtarm[9]['linkpos'])
    # pg.plotAxisSelf(hrp5nmnp, hrp5nrobot.rgtarm[9]['linkend'], hrp5nrobotrgtarmlj9_rotmat)
    hrp5nrobotrgthnd.setMat(pandanpmat4 = hrp5nrobotrgtarmlj9_rotmat)
    hrp5nrobotrgthnd.reparentTo(hrp5nmnp)
    if jawwidthrgt is not None:
        hrp5nrobotrgthnd.setJawwidth(jawwidthrgt)

    # lfthnd
    hrp5nrobotlfthnd = handpkg.newHand('lft')
    hrp5nrobotlftarmlj9_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[9]['rotmat'], hrp5nrobot.lftarm[9]['linkpos'])
    # pg.plotAxisSelf(hrp5nmnp, hrp5nrobot.lftarm[9]['linkend'], hrp5nrobotlftarmlj9_rotmat)
    hrp5nrobotlfthnd.setMat(pandanpmat4 = hrp5nrobotlftarmlj9_rotmat)
    hrp5nrobotlfthnd.reparentTo(hrp5nmnp)
    if jawwidthlft is not None:
        hrp5nrobotlfthnd.setJawwidth(jawwidthlft)

    return hrp5nmnp


def genmnplist(hrp5nrobot, handpkg, jawwidthrgt = None, jawwidthlft = None):
    """
    generate a list of panda3d nodepath for the hrp5robot
    mnp indicates this function generates a mesh nodepath

    return [[right arm mnp list], [leftarm mnp list], [body mnp list]]
    where one arm mnp list is ordered from base to end-effector

    :param hrp5robot: the Hrp5Robot object, see hrp5.py
    :return: a nodepath which is ready to be plotted using plotmesh

    author: weiwei
    date: 20170608
    """

    hrp5nmnp = NodePath("hrp5nmnp")

    this_dir, this_filename = os.path.split(__file__)

    # chest0-2, head1 (neck is not plotted)
    hrp5nbody_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Body.egg"))
    hrp5nchest0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Chest_Link0.egg"))
    hrp5nchest1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Chest_Link1.egg"))
    hrp5nchest2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Chest_Link2.egg"))
    hrp5nhead0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Head_Link0.egg"))
    hrp5nhead1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Head_Link1.egg"))

    hrp5nbody_model  = loader.loadModel(hrp5nbody_filepath)
    hrp5nchest0_model  = loader.loadModel(hrp5nchest0_filepath)
    hrp5nchest1_model = loader.loadModel(hrp5nchest1_filepath)
    hrp5nchest2_model = loader.loadModel(hrp5nchest2_filepath)
    hrp5nhead0_model = loader.loadModel(hrp5nhead0_filepath)
    hrp5nhead1_model = loader.loadModel(hrp5nhead1_filepath)

    hrp5nbody_nodepath = NodePath("hrp5nbody")
    hrp5nchest0_nodepath = NodePath("hrp5nchest0")
    hrp5nchest1_nodepath = NodePath("hrp5nchest1")
    hrp5nchest2_nodepath = NodePath("hrp5nchest2")
    hrp5nhead0_nodepath = NodePath("hrp5nhead0")
    hrp5nhead1_nodepath = NodePath("hrp5nhead1")

    # body
    hrp5nbody_model.instanceTo(hrp5nbody_nodepath)
    hrp5nbody_rotmat = Mat4.identMat()
    hrp5nbody_nodepath.setMat(hrp5nbody_rotmat)
    hrp5nbody_nodepath.setZ(0)
    # chest
    hrp5nchest0_model.instanceTo(hrp5nchest0_nodepath)
    hrp5nchest0_rotmat = Mat4.identMat()
    hrp5nchest0_nodepath.setMat(hrp5nchest0_rotmat)
    hrp5nchest0_nodepath.setZ(274)
    hrp5nchest1_model.instanceTo(hrp5nchest1_nodepath)
    hrp5nchest1_nodepath.setColor(.7,.7,.2,1)
    hrp5nchest2_model.instanceTo(hrp5nchest2_nodepath)
    hrp5nchest2_rotmat = pg.cvtMat4(hrp5nrobot.base['rotmat'])
    hrp5nchest2_nodepath.setMat(hrp5nchest2_rotmat)
    hrp5nchest2_nodepath.setColor(.7,.7,.7,1)
    hrp5nhead0_model.instanceTo(hrp5nhead0_nodepath)
    hrp5nhead0_nodepath.setH(hrp5nrobot.initjnts[0])
    hrp5nhead0_nodepath.setX(32)
    hrp5nhead0_nodepath.setZ(521)
    hrp5nhead1_model.instanceTo(hrp5nhead1_nodepath)
    hrp5nhead1_nodepath.setP(hrp5nrobot.initjnts[1])
    hrp5nhead1_nodepath.setX(40)
    hrp5nhead1_nodepath.setColor(.5,.5,.5,1)

    hrp5nbody_nodepath.reparentTo(hrp5nmnp)
    hrp5nchest0_nodepath.reparentTo(hrp5nbody_nodepath)
    hrp5nchest1_nodepath.reparentTo(hrp5nchest0_nodepath)
    hrp5nchest2_nodepath.reparentTo(hrp5nchest1_nodepath)
    hrp5nhead0_nodepath.reparentTo(hrp5nchest2_nodepath)
    hrp5nhead1_nodepath.reparentTo(hrp5nhead0_nodepath)

    # rgtarm
    hrp5nrgtarmlj0_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rscap_Link0.egg"))
    hrp5nrgtarmlj0_model = loader.loadModel(hrp5nrgtarmlj0_filepath)
    hrp5nrgtarmlj0_nodepath = NodePath("nxtrgtarmlj0_nodepath")
    hrp5nrgtarmlj0_model.instanceTo(hrp5nrgtarmlj0_nodepath)
    hrp5nrgtarmlj0_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[1]['rotmat'], hrp5nrobot.rgtarm[1]['linkpos'])
    hrp5nrgtarmlj0_nodepath.setMat(hrp5nrgtarmlj0_rotmat)
    hrp5nrgtarmlj0_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj0_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj1_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link0.egg"))
    hrp5nrgtarmlj1_model = loader.loadModel(hrp5nrgtarmlj1_filepath)
    hrp5nrgtarmlj1_nodepath = NodePath("nxtrgtarmlj1_nodepath")
    hrp5nrgtarmlj1_model.instanceTo(hrp5nrgtarmlj1_nodepath)
    hrp5nrgtarmlj1_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[2]['rotmat'], hrp5nrobot.rgtarm[2]['linkpos'])
    hrp5nrgtarmlj1_nodepath.setMat(hrp5nrgtarmlj1_rotmat)
    hrp5nrgtarmlj1_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj1_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj2_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link1.egg"))
    hrp5nrgtarmlj2_model = loader.loadModel(hrp5nrgtarmlj2_filepath)
    hrp5nrgtarmlj2_nodepath = NodePath("nxtrgtarmlj2_nodepath")
    hrp5nrgtarmlj2_model.instanceTo(hrp5nrgtarmlj2_nodepath)
    hrp5nrgtarmlj2_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[3]['rotmat'], hrp5nrobot.rgtarm[3]['linkpos'])
    hrp5nrgtarmlj2_nodepath.setMat(hrp5nrgtarmlj2_rotmat)
    hrp5nrgtarmlj2_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj2_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj3_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link2.egg"))
    hrp5nrgtarmlj3_model = loader.loadModel(hrp5nrgtarmlj3_filepath)
    hrp5nrgtarmlj3_nodepath = NodePath("nxtrgtarmlj3_nodepath")
    hrp5nrgtarmlj3_model.instanceTo(hrp5nrgtarmlj3_nodepath)
    hrp5nrgtarmlj3_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[4]['rotmat'], hrp5nrobot.rgtarm[4]['linkpos'])
    hrp5nrgtarmlj3_nodepath.setMat(hrp5nrgtarmlj3_rotmat)
    hrp5nrgtarmlj3_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj3_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj4_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link3.egg"))
    hrp5nrgtarmlj4_model = loader.loadModel(hrp5nrgtarmlj4_filepath)
    hrp5nrgtarmlj4_nodepath = NodePath("nxtrgtarmlj4_nodepath")
    hrp5nrgtarmlj4_model.instanceTo(hrp5nrgtarmlj4_nodepath)
    hrp5nrgtarmlj4_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[5]['rotmat'], hrp5nrobot.rgtarm[5]['linkpos'])
    hrp5nrgtarmlj4_nodepath.setMat(hrp5nrgtarmlj4_rotmat)
    hrp5nrgtarmlj4_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj4_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj5_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link4.egg"))
    hrp5nrgtarmlj5_model = loader.loadModel(hrp5nrgtarmlj5_filepath)
    hrp5nrgtarmlj5_nodepath = NodePath("nxtrgtarmlj5_nodepath")
    hrp5nrgtarmlj5_model.instanceTo(hrp5nrgtarmlj5_nodepath)
    hrp5nrgtarmlj5_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[6]['rotmat'], hrp5nrobot.rgtarm[6]['linkpos'])
    hrp5nrgtarmlj5_nodepath.setMat(hrp5nrgtarmlj5_rotmat)
    hrp5nrgtarmlj5_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj5_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj6_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link5.egg"))
    hrp5nrgtarmlj6_model = loader.loadModel(hrp5nrgtarmlj6_filepath)
    hrp5nrgtarmlj6_nodepath = NodePath("nxtrgtarmlj6_nodepath")
    hrp5nrgtarmlj6_model.instanceTo(hrp5nrgtarmlj6_nodepath)
    hrp5nrgtarmlj6_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[7]['rotmat'], hrp5nrobot.rgtarm[7]['linkpos'])
    hrp5nrgtarmlj6_nodepath.setMat(hrp5nrgtarmlj6_rotmat)
    hrp5nrgtarmlj6_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj6_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj7_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link6.egg"))
    hrp5nrgtarmlj7_model = loader.loadModel(hrp5nrgtarmlj7_filepath)
    hrp5nrgtarmlj7_nodepath = NodePath("nxtrgtarmlj7_nodepath")
    hrp5nrgtarmlj7_model.instanceTo(hrp5nrgtarmlj7_nodepath)
    hrp5nrgtarmlj7_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[8]['rotmat'], hrp5nrobot.rgtarm[8]['linkpos'])
    hrp5nrgtarmlj7_nodepath.setMat(hrp5nrgtarmlj7_rotmat)
    hrp5nrgtarmlj7_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj7_nodepath.reparentTo(hrp5nmnp)

    # lftarm
    hrp5nlftarmlj0_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Lscap_Link0.egg"))
    hrp5nlftarmlj0_model = loader.loadModel(hrp5nlftarmlj0_filepath)
    hrp5nlftarmlj0_nodepath = NodePath("nxtlftarmlj0_nodepath")
    hrp5nlftarmlj0_model.instanceTo(hrp5nlftarmlj0_nodepath)
    hrp5nlftarmlj0_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[1]['rotmat'], hrp5nrobot.lftarm[1]['linkpos'])
    hrp5nlftarmlj0_nodepath.setMat(hrp5nlftarmlj0_rotmat)
    hrp5nlftarmlj0_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj0_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj1_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link0.egg"))
    hrp5nlftarmlj1_model = loader.loadModel(hrp5nlftarmlj1_filepath)
    hrp5nlftarmlj1_nodepath = NodePath("nxtlftarmlj1_nodepath")
    hrp5nlftarmlj1_model.instanceTo(hrp5nlftarmlj1_nodepath)
    hrp5nlftarmlj1_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[2]['rotmat'], hrp5nrobot.lftarm[2]['linkpos'])
    hrp5nlftarmlj1_nodepath.setMat(hrp5nlftarmlj1_rotmat)
    hrp5nlftarmlj1_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj1_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj2_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link1.egg"))
    hrp5nlftarmlj2_model = loader.loadModel(hrp5nlftarmlj2_filepath)
    hrp5nlftarmlj2_nodepath = NodePath("nxtlftarmlj2_nodepath")
    hrp5nlftarmlj2_model.instanceTo(hrp5nlftarmlj2_nodepath)
    hrp5nlftarmlj2_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[3]['rotmat'], hrp5nrobot.lftarm[3]['linkpos'])
    hrp5nlftarmlj2_nodepath.setMat(hrp5nlftarmlj2_rotmat)
    hrp5nlftarmlj2_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj2_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj3_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link2.egg"))
    hrp5nlftarmlj3_model = loader.loadModel(hrp5nlftarmlj3_filepath)
    hrp5nlftarmlj3_nodepath = NodePath("nxtlftarmlj3_nodepath")
    hrp5nlftarmlj3_model.instanceTo(hrp5nlftarmlj3_nodepath)
    hrp5nlftarmlj3_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[4]['rotmat'], hrp5nrobot.lftarm[4]['linkpos'])
    hrp5nlftarmlj3_nodepath.setMat(hrp5nlftarmlj3_rotmat)
    hrp5nlftarmlj3_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj3_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj4_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link3.egg"))
    hrp5nlftarmlj4_model = loader.loadModel(hrp5nlftarmlj4_filepath)
    hrp5nlftarmlj4_nodepath = NodePath("nxtlftarmlj4_nodepath")
    hrp5nlftarmlj4_model.instanceTo(hrp5nlftarmlj4_nodepath)
    hrp5nlftarmlj4_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[5]['rotmat'], hrp5nrobot.lftarm[5]['linkpos'])
    hrp5nlftarmlj4_nodepath.setMat(hrp5nlftarmlj4_rotmat)
    hrp5nlftarmlj4_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj4_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj5_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link4.egg"))
    hrp5nlftarmlj5_model = loader.loadModel(hrp5nlftarmlj5_filepath)
    hrp5nlftarmlj5_nodepath = NodePath("nxtlftarmlj5_nodepath")
    hrp5nlftarmlj5_model.instanceTo(hrp5nlftarmlj5_nodepath)
    hrp5nlftarmlj5_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[6]['rotmat'], hrp5nrobot.lftarm[6]['linkpos'])
    hrp5nlftarmlj5_nodepath.setMat(hrp5nlftarmlj5_rotmat)
    hrp5nlftarmlj5_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj5_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj6_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link5.egg"))
    hrp5nlftarmlj6_model = loader.loadModel(hrp5nlftarmlj6_filepath)
    hrp5nlftarmlj6_nodepath = NodePath("nxtlftarmlj6_nodepath")
    hrp5nlftarmlj6_model.instanceTo(hrp5nlftarmlj6_nodepath)
    hrp5nlftarmlj6_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[7]['rotmat'], hrp5nrobot.lftarm[7]['linkpos'])
    hrp5nlftarmlj6_nodepath.setMat(hrp5nlftarmlj6_rotmat)
    hrp5nlftarmlj6_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj6_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj7_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link6.egg"))
    hrp5nlftarmlj7_model = loader.loadModel(hrp5nlftarmlj7_filepath)
    hrp5nlftarmlj7_nodepath = NodePath("nxtlftarmlj7_nodepath")
    hrp5nlftarmlj7_model.instanceTo(hrp5nlftarmlj7_nodepath)
    hrp5nlftarmlj7_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[8]['rotmat'], hrp5nrobot.lftarm[8]['linkpos'])
    hrp5nlftarmlj7_nodepath.setMat(hrp5nlftarmlj7_rotmat)
    hrp5nlftarmlj7_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj7_nodepath.reparentTo(hrp5nmnp)

    # rgthnd
    hrp5nrobotrgthnd = handpkg.newHand('rgt')
    hrp5nrobotrgtarmlj9_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[9]['rotmat'], hrp5nrobot.rgtarm[9]['linkpos'])
    # pg.plotAxisSelf(hrp5nmnp, hrp5nrobot.rgtarm[9]['linkend'], hrp5nrobotrgtarmlj9_rotmat)
    hrp5nrobotrgthnd.setMat(pandanpmat4 = hrp5nrobotrgtarmlj9_rotmat)
    hrp5nrobotrgthnd.reparentTo(hrp5nmnp)
    if jawwidthrgt is not None:
        hrp5nrobotrgthnd.setJawwidth(jawwidthrgt)

    # lfthnd
    hrp5nrobotlfthnd = handpkg.newHand('lft')
    hrp5nrobotlftarmlj9_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[9]['rotmat'], hrp5nrobot.lftarm[9]['linkpos'])
    # pg.plotAxisSelf(hrp5nmnp, hrp5nrobot.lftarm[9]['linkend'], hrp5nrobotlftarmlj9_rotmat)
    hrp5nrobotlfthnd.setMat(pandanpmat4 = hrp5nrobotlftarmlj9_rotmat)
    hrp5nrobotlfthnd.reparentTo(hrp5nmnp)
    if jawwidthlft is not None:
        hrp5nrobotlfthnd.setJawwidth(jawwidthlft)

    return [[hrp5nrgtarmlj0_nodepath, hrp5nrgtarmlj1_nodepath, hrp5nrgtarmlj2_nodepath,
             hrp5nrgtarmlj3_nodepath, hrp5nrgtarmlj4_nodepath, hrp5nrgtarmlj5_nodepath,
             hrp5nrgtarmlj6_nodepath, hrp5nrgtarmlj7_nodepath, hrp5nrobotrgthnd.handnp],
            [hrp5nlftarmlj0_nodepath, hrp5nlftarmlj1_nodepath, hrp5nlftarmlj2_nodepath,
             hrp5nlftarmlj3_nodepath, hrp5nlftarmlj4_nodepath, hrp5nlftarmlj5_nodepath,
             hrp5nlftarmlj6_nodepath, hrp5nlftarmlj7_nodepath, hrp5nrobotlfthnd.handnp],
            [hrp5nbody_nodepath]]


def genmnp_nm(hrp5nrobot, handpkg, jawwidthrgt = None, jawwidthlft = None):
    """
    generate a panda3d nodepath for the hrp5robot
    mnp indicates this function generates a mesh nodepath

    :param hrp5robot: the Hrp5Robot object, see hrp5.py
    :return: a nodepath which is ready to be plotted using plotmesh

    author: weiwei
    date: 20161202
    """

    hrp5nmnp = NodePath("hrp5nmnp")

    this_dir, this_filename = os.path.split(__file__)

    # chest0-2, head1 (neck is not plotted)
    hrp5nbody_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg/nomat", "HRP-5P_ConceptDesign_Body.egg"))
    hrp5nchest0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg/nomat", "HRP-5P_ConceptDesign_Chest_Link0.egg"))
    hrp5nchest1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg/nomat", "HRP-5P_ConceptDesign_Chest_Link1.egg"))
    hrp5nchest2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg/nomat", "HRP-5P_ConceptDesign_Chest_Link2.egg"))
    hrp5nhead0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg/nomat", "HRP-5P_ConceptDesign_Head_Link0.egg"))
    hrp5nhead1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg/nomat", "HRP-5P_ConceptDesign_Head_Link1.egg"))

    hrp5nbody_model  = loader.loadModel(hrp5nbody_filepath)
    hrp5nchest0_model  = loader.loadModel(hrp5nchest0_filepath)
    hrp5nchest1_model = loader.loadModel(hrp5nchest1_filepath)
    hrp5nchest2_model = loader.loadModel(hrp5nchest2_filepath)
    hrp5nhead0_model = loader.loadModel(hrp5nhead0_filepath)
    hrp5nhead1_model = loader.loadModel(hrp5nhead1_filepath)

    hrp5nbody_nodepath = NodePath("hrp5nbody")
    hrp5nchest0_nodepath = NodePath("hrp5nchest0")
    hrp5nchest1_nodepath = NodePath("hrp5nchest1")
    hrp5nchest2_nodepath = NodePath("hrp5nchest2")
    hrp5nhead0_nodepath = NodePath("hrp5nhead0")
    hrp5nhead1_nodepath = NodePath("hrp5nhead1")

    # body
    hrp5nbody_model.instanceTo(hrp5nbody_nodepath)
    hrp5nbody_rotmat = Mat4.identMat()
    hrp5nbody_nodepath.setMat(hrp5nbody_rotmat)
    hrp5nbody_nodepath.setZ(0)
    # chest
    hrp5nchest0_model.instanceTo(hrp5nchest0_nodepath)
    hrp5nchest0_rotmat = Mat4.identMat()
    hrp5nchest0_nodepath.setMat(hrp5nchest0_rotmat)
    hrp5nchest0_nodepath.setZ(274)
    hrp5nchest1_model.instanceTo(hrp5nchest1_nodepath)
    hrp5nchest1_nodepath.setColor(.7,.7,.2,1)
    hrp5nchest2_model.instanceTo(hrp5nchest2_nodepath)
    hrp5nchest2_rotmat = pg.cvtMat4(hrp5nrobot.base['rotmat'])
    hrp5nchest2_nodepath.setMat(hrp5nchest2_rotmat)
    hrp5nchest2_nodepath.setColor(.7,.7,.7,1)
    hrp5nhead0_model.instanceTo(hrp5nhead0_nodepath)
    hrp5nhead0_nodepath.setH(hrp5nrobot.initjnts[0])
    hrp5nhead0_nodepath.setX(32)
    hrp5nhead0_nodepath.setZ(521)
    hrp5nhead1_model.instanceTo(hrp5nhead1_nodepath)
    hrp5nhead1_nodepath.setP(hrp5nrobot.initjnts[1])
    hrp5nhead1_nodepath.setX(40)
    hrp5nhead1_nodepath.setColor(.5,.5,.5,1)

    hrp5nbody_nodepath.reparentTo(hrp5nmnp)
    hrp5nchest0_nodepath.reparentTo(hrp5nbody_nodepath)
    hrp5nchest1_nodepath.reparentTo(hrp5nchest0_nodepath)
    hrp5nchest2_nodepath.reparentTo(hrp5nchest1_nodepath)
    hrp5nhead0_nodepath.reparentTo(hrp5nchest2_nodepath)
    hrp5nhead1_nodepath.reparentTo(hrp5nhead0_nodepath)

    # rgtarm
    hrp5nrgtarmlj0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rscap_Link0.egg"))
    hrp5nrgtarmlj0_model = loader.loadModel(hrp5nrgtarmlj0_filepath)
    hrp5nrgtarmlj0_nodepath = NodePath("nxtrgtarmlj0_nodepath")
    hrp5nrgtarmlj0_model.instanceTo(hrp5nrgtarmlj0_nodepath)
    hrp5nrgtarmlj0_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[1]['rotmat'], hrp5nrobot.rgtarm[1]['linkpos'])
    hrp5nrgtarmlj0_nodepath.setMat(hrp5nrgtarmlj0_rotmat)
    hrp5nrgtarmlj0_nodepath.setColor(.5,.5,.5,1)
    hrp5nrgtarmlj0_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link0.egg"))
    hrp5nrgtarmlj1_model = loader.loadModel(hrp5nrgtarmlj1_filepath)
    hrp5nrgtarmlj1_nodepath = NodePath("nxtrgtarmlj1_nodepath")
    hrp5nrgtarmlj1_model.instanceTo(hrp5nrgtarmlj1_nodepath)
    hrp5nrgtarmlj1_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[2]['rotmat'], hrp5nrobot.rgtarm[2]['linkpos'])
    hrp5nrgtarmlj1_nodepath.setMat(hrp5nrgtarmlj1_rotmat)
    hrp5nrgtarmlj1_nodepath.setColor(.5,.5,.5,1)
    hrp5nrgtarmlj1_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link1.egg"))
    hrp5nrgtarmlj2_model = loader.loadModel(hrp5nrgtarmlj2_filepath)
    hrp5nrgtarmlj2_nodepath = NodePath("nxtrgtarmlj2_nodepath")
    hrp5nrgtarmlj2_model.instanceTo(hrp5nrgtarmlj2_nodepath)
    hrp5nrgtarmlj2_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[3]['rotmat'], hrp5nrobot.rgtarm[3]['linkpos'])
    hrp5nrgtarmlj2_nodepath.setMat(hrp5nrgtarmlj2_rotmat)
    hrp5nrgtarmlj2_nodepath.setColor(.5,.5,.5,1)
    hrp5nrgtarmlj2_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link2.egg"))
    hrp5nrgtarmlj3_model = loader.loadModel(hrp5nrgtarmlj3_filepath)
    hrp5nrgtarmlj3_nodepath = NodePath("nxtrgtarmlj3_nodepath")
    hrp5nrgtarmlj3_model.instanceTo(hrp5nrgtarmlj3_nodepath)
    hrp5nrgtarmlj3_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[4]['rotmat'], hrp5nrobot.rgtarm[4]['linkpos'])
    hrp5nrgtarmlj3_nodepath.setMat(hrp5nrgtarmlj3_rotmat)
    hrp5nrgtarmlj3_nodepath.setColor(.5,.5,.5,1)
    hrp5nrgtarmlj3_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj4_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link3.egg"))
    hrp5nrgtarmlj4_model = loader.loadModel(hrp5nrgtarmlj4_filepath)
    hrp5nrgtarmlj4_nodepath = NodePath("nxtrgtarmlj4_nodepath")
    hrp5nrgtarmlj4_model.instanceTo(hrp5nrgtarmlj4_nodepath)
    hrp5nrgtarmlj4_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[5]['rotmat'], hrp5nrobot.rgtarm[5]['linkpos'])
    hrp5nrgtarmlj4_nodepath.setMat(hrp5nrgtarmlj4_rotmat)
    hrp5nrgtarmlj4_nodepath.setColor(.5,.5,.5,1)
    hrp5nrgtarmlj4_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj5_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link4.egg"))
    hrp5nrgtarmlj5_model = loader.loadModel(hrp5nrgtarmlj5_filepath)
    hrp5nrgtarmlj5_nodepath = NodePath("nxtrgtarmlj5_nodepath")
    hrp5nrgtarmlj5_model.instanceTo(hrp5nrgtarmlj5_nodepath)
    hrp5nrgtarmlj5_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[6]['rotmat'], hrp5nrobot.rgtarm[6]['linkpos'])
    hrp5nrgtarmlj5_nodepath.setMat(hrp5nrgtarmlj5_rotmat)
    hrp5nrgtarmlj5_nodepath.setColor(.5,.5,.5,1)
    hrp5nrgtarmlj5_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj6_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link5.egg"))
    hrp5nrgtarmlj6_model = loader.loadModel(hrp5nrgtarmlj6_filepath)
    hrp5nrgtarmlj6_nodepath = NodePath("nxtrgtarmlj6_nodepath")
    hrp5nrgtarmlj6_model.instanceTo(hrp5nrgtarmlj6_nodepath)
    hrp5nrgtarmlj6_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[7]['rotmat'], hrp5nrobot.rgtarm[7]['linkpos'])
    hrp5nrgtarmlj6_nodepath.setMat(hrp5nrgtarmlj6_rotmat)
    hrp5nrgtarmlj6_nodepath.setColor(.5,.5,.5,1)
    hrp5nrgtarmlj6_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj7_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link6.egg"))
    hrp5nrgtarmlj7_model = loader.loadModel(hrp5nrgtarmlj7_filepath)
    hrp5nrgtarmlj7_nodepath = NodePath("nxtrgtarmlj7_nodepath")
    hrp5nrgtarmlj7_model.instanceTo(hrp5nrgtarmlj7_nodepath)
    hrp5nrgtarmlj7_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[8]['rotmat'], hrp5nrobot.rgtarm[8]['linkpos'])
    hrp5nrgtarmlj7_nodepath.setMat(hrp5nrgtarmlj7_rotmat)
    hrp5nrgtarmlj7_nodepath.setColor(.5,.5,.5,1)
    hrp5nrgtarmlj7_nodepath.reparentTo(hrp5nmnp)
    #
    # rgtarm
    hrp5nrgtarmlj0_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rscap_Link0.egg"))
    hrp5nrgtarmlj0_model = loader.loadModel(hrp5nrgtarmlj0_filepath)
    hrp5nrgtarmlj0_nodepath = NodePath("nxtrgtarmlj0_nodepath")
    hrp5nrgtarmlj0_model.instanceTo(hrp5nrgtarmlj0_nodepath)
    hrp5nrgtarmlj0_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[1]['rotmat'], hrp5nrobot.rgtarm[1]['linkpos'])
    hrp5nrgtarmlj0_nodepath.setMat(hrp5nrgtarmlj0_rotmat)
    hrp5nrgtarmlj0_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj0_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj1_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link0.egg"))
    hrp5nrgtarmlj1_model = loader.loadModel(hrp5nrgtarmlj1_filepath)
    hrp5nrgtarmlj1_nodepath = NodePath("nxtrgtarmlj1_nodepath")
    hrp5nrgtarmlj1_model.instanceTo(hrp5nrgtarmlj1_nodepath)
    hrp5nrgtarmlj1_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[2]['rotmat'], hrp5nrobot.rgtarm[2]['linkpos'])
    hrp5nrgtarmlj1_nodepath.setMat(hrp5nrgtarmlj1_rotmat)
    hrp5nrgtarmlj1_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj1_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj2_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link1.egg"))
    hrp5nrgtarmlj2_model = loader.loadModel(hrp5nrgtarmlj2_filepath)
    hrp5nrgtarmlj2_nodepath = NodePath("nxtrgtarmlj2_nodepath")
    hrp5nrgtarmlj2_model.instanceTo(hrp5nrgtarmlj2_nodepath)
    hrp5nrgtarmlj2_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[3]['rotmat'], hrp5nrobot.rgtarm[3]['linkpos'])
    hrp5nrgtarmlj2_nodepath.setMat(hrp5nrgtarmlj2_rotmat)
    hrp5nrgtarmlj2_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj2_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj3_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link2.egg"))
    hrp5nrgtarmlj3_model = loader.loadModel(hrp5nrgtarmlj3_filepath)
    hrp5nrgtarmlj3_nodepath = NodePath("nxtrgtarmlj3_nodepath")
    hrp5nrgtarmlj3_model.instanceTo(hrp5nrgtarmlj3_nodepath)
    hrp5nrgtarmlj3_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[4]['rotmat'], hrp5nrobot.rgtarm[4]['linkpos'])
    hrp5nrgtarmlj3_nodepath.setMat(hrp5nrgtarmlj3_rotmat)
    hrp5nrgtarmlj3_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj3_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj4_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link3.egg"))
    hrp5nrgtarmlj4_model = loader.loadModel(hrp5nrgtarmlj4_filepath)
    hrp5nrgtarmlj4_nodepath = NodePath("nxtrgtarmlj4_nodepath")
    hrp5nrgtarmlj4_model.instanceTo(hrp5nrgtarmlj4_nodepath)
    hrp5nrgtarmlj4_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[5]['rotmat'], hrp5nrobot.rgtarm[5]['linkpos'])
    hrp5nrgtarmlj4_nodepath.setMat(hrp5nrgtarmlj4_rotmat)
    hrp5nrgtarmlj4_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj4_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj5_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link4.egg"))
    hrp5nrgtarmlj5_model = loader.loadModel(hrp5nrgtarmlj5_filepath)
    hrp5nrgtarmlj5_nodepath = NodePath("nxtrgtarmlj5_nodepath")
    hrp5nrgtarmlj5_model.instanceTo(hrp5nrgtarmlj5_nodepath)
    hrp5nrgtarmlj5_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[6]['rotmat'], hrp5nrobot.rgtarm[6]['linkpos'])
    hrp5nrgtarmlj5_nodepath.setMat(hrp5nrgtarmlj5_rotmat)
    hrp5nrgtarmlj5_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj5_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj6_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link5.egg"))
    hrp5nrgtarmlj6_model = loader.loadModel(hrp5nrgtarmlj6_filepath)
    hrp5nrgtarmlj6_nodepath = NodePath("nxtrgtarmlj6_nodepath")
    hrp5nrgtarmlj6_model.instanceTo(hrp5nrgtarmlj6_nodepath)
    hrp5nrgtarmlj6_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[7]['rotmat'], hrp5nrobot.rgtarm[7]['linkpos'])
    hrp5nrgtarmlj6_nodepath.setMat(hrp5nrgtarmlj6_rotmat)
    hrp5nrgtarmlj6_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj6_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nrgtarmlj7_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link6.egg"))
    hrp5nrgtarmlj7_model = loader.loadModel(hrp5nrgtarmlj7_filepath)
    hrp5nrgtarmlj7_nodepath = NodePath("nxtrgtarmlj7_nodepath")
    hrp5nrgtarmlj7_model.instanceTo(hrp5nrgtarmlj7_nodepath)
    hrp5nrgtarmlj7_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[8]['rotmat'], hrp5nrobot.rgtarm[8]['linkpos'])
    hrp5nrgtarmlj7_nodepath.setMat(hrp5nrgtarmlj7_rotmat)
    hrp5nrgtarmlj7_nodepath.setColor(.5, .5, .5, 1)
    hrp5nrgtarmlj7_nodepath.reparentTo(hrp5nmnp)

    # lftarm
    hrp5nlftarmlj0_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Lscap_Link0.egg"))
    hrp5nlftarmlj0_model = loader.loadModel(hrp5nlftarmlj0_filepath)
    hrp5nlftarmlj0_nodepath = NodePath("nxtlftarmlj0_nodepath")
    hrp5nlftarmlj0_model.instanceTo(hrp5nlftarmlj0_nodepath)
    hrp5nlftarmlj0_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[1]['rotmat'], hrp5nrobot.lftarm[1]['linkpos'])
    hrp5nlftarmlj0_nodepath.setMat(hrp5nlftarmlj0_rotmat)
    hrp5nlftarmlj0_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj0_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj1_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link0.egg"))
    hrp5nlftarmlj1_model = loader.loadModel(hrp5nlftarmlj1_filepath)
    hrp5nlftarmlj1_nodepath = NodePath("nxtlftarmlj1_nodepath")
    hrp5nlftarmlj1_model.instanceTo(hrp5nlftarmlj1_nodepath)
    hrp5nlftarmlj1_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[2]['rotmat'], hrp5nrobot.lftarm[2]['linkpos'])
    hrp5nlftarmlj1_nodepath.setMat(hrp5nlftarmlj1_rotmat)
    hrp5nlftarmlj1_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj1_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj2_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link1.egg"))
    hrp5nlftarmlj2_model = loader.loadModel(hrp5nlftarmlj2_filepath)
    hrp5nlftarmlj2_nodepath = NodePath("nxtlftarmlj2_nodepath")
    hrp5nlftarmlj2_model.instanceTo(hrp5nlftarmlj2_nodepath)
    hrp5nlftarmlj2_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[3]['rotmat'], hrp5nrobot.lftarm[3]['linkpos'])
    hrp5nlftarmlj2_nodepath.setMat(hrp5nlftarmlj2_rotmat)
    hrp5nlftarmlj2_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj2_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj3_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link2.egg"))
    hrp5nlftarmlj3_model = loader.loadModel(hrp5nlftarmlj3_filepath)
    hrp5nlftarmlj3_nodepath = NodePath("nxtlftarmlj3_nodepath")
    hrp5nlftarmlj3_model.instanceTo(hrp5nlftarmlj3_nodepath)
    hrp5nlftarmlj3_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[4]['rotmat'], hrp5nrobot.lftarm[4]['linkpos'])
    hrp5nlftarmlj3_nodepath.setMat(hrp5nlftarmlj3_rotmat)
    hrp5nlftarmlj3_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj3_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj4_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link3.egg"))
    hrp5nlftarmlj4_model = loader.loadModel(hrp5nlftarmlj4_filepath)
    hrp5nlftarmlj4_nodepath = NodePath("nxtlftarmlj4_nodepath")
    hrp5nlftarmlj4_model.instanceTo(hrp5nlftarmlj4_nodepath)
    hrp5nlftarmlj4_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[5]['rotmat'], hrp5nrobot.lftarm[5]['linkpos'])
    hrp5nlftarmlj4_nodepath.setMat(hrp5nlftarmlj4_rotmat)
    hrp5nlftarmlj4_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj4_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj5_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link4.egg"))
    hrp5nlftarmlj5_model = loader.loadModel(hrp5nlftarmlj5_filepath)
    hrp5nlftarmlj5_nodepath = NodePath("nxtlftarmlj5_nodepath")
    hrp5nlftarmlj5_model.instanceTo(hrp5nlftarmlj5_nodepath)
    hrp5nlftarmlj5_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[6]['rotmat'], hrp5nrobot.lftarm[6]['linkpos'])
    hrp5nlftarmlj5_nodepath.setMat(hrp5nlftarmlj5_rotmat)
    hrp5nlftarmlj5_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj5_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj6_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link5.egg"))
    hrp5nlftarmlj6_model = loader.loadModel(hrp5nlftarmlj6_filepath)
    hrp5nlftarmlj6_nodepath = NodePath("nxtlftarmlj6_nodepath")
    hrp5nlftarmlj6_model.instanceTo(hrp5nlftarmlj6_nodepath)
    hrp5nlftarmlj6_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[7]['rotmat'], hrp5nrobot.lftarm[7]['linkpos'])
    hrp5nlftarmlj6_nodepath.setMat(hrp5nlftarmlj6_rotmat)
    hrp5nlftarmlj6_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj6_nodepath.reparentTo(hrp5nmnp)
    #
    hrp5nlftarmlj7_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link6.egg"))
    hrp5nlftarmlj7_model = loader.loadModel(hrp5nlftarmlj7_filepath)
    hrp5nlftarmlj7_nodepath = NodePath("nxtlftarmlj7_nodepath")
    hrp5nlftarmlj7_model.instanceTo(hrp5nlftarmlj7_nodepath)
    hrp5nlftarmlj7_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[8]['rotmat'], hrp5nrobot.lftarm[8]['linkpos'])
    hrp5nlftarmlj7_nodepath.setMat(hrp5nlftarmlj7_rotmat)
    hrp5nlftarmlj7_nodepath.setColor(.5, .5, .5, 1)
    hrp5nlftarmlj7_nodepath.reparentTo(hrp5nmnp)

    # rgthnd
    hrp5nrobotrgthnd = handpkg.newHand('rgt')
    hrp5nrobotrgtarmlj9_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[9]['rotmat'], hrp5nrobot.rgtarm[9]['linkpos'])
    # pg.plotAxisSelf(hrp5nmnp, hrp5nrobot.rgtarm[9]['linkend'], hrp5nrobotrgtarmlj9_rotmat)
    hrp5nrobotrgthnd.setMat(pandanpmat4 = hrp5nrobotrgtarmlj9_rotmat)
    hrp5nrobotrgthnd.reparentTo(hrp5nmnp)
    if jawwidthrgt is not None:
        hrp5nrobotrgthnd.setJawwidth(jawwidthrgt)

    # lfthnd
    hrp5nrobotlfthnd = handpkg.newHand('lft')
    hrp5nrobotlftarmlj9_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[9]['rotmat'], hrp5nrobot.lftarm[9]['linkpos'])
    # pg.plotAxisSelf(hrp5nmnp, hrp5nrobot.lftarm[9]['linkend'], hrp5nrobotlftarmlj9_rotmat)
    hrp5nrobotlfthnd.setMat(pandanpmat4 = hrp5nrobotlftarmlj9_rotmat)
    hrp5nrobotlfthnd.reparentTo(hrp5nmnp)
    if jawwidthlft is not None:
        hrp5nrobotlfthnd.setJawwidth(jawwidthlft)

    return hrp5nmnp