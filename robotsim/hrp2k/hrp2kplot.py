import os

import numpy as np
from manipulation.grip.robotiq85 import rtq85
from manipulation.grip.hrp5three import hrp5three
from panda3d.core import *

import pandaplotutils.pandageom as pg

# TODO, scale up the models for collision detection

def plotstick(pandanp, hrp2krobot, rgtrbga=np.array([.5 ,0 ,0 ,0]), lftrgba=np.array([.5 ,0 ,0 ,0])):
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
        pg.plotDumbbell(pandanp, spos=hrp2krobot.rgtarm[i]['linkpos'], epos=hrp2krobot.rgtarm[i]['linkend'],
                               thickness=20, rgba=rgtrbga)
        i = hrp2krobot.rgtarm[i]['child']
    i = 0
    while i != -1:
        pg.plotDumbbell(pandanp, spos=hrp2krobot.lftarm[i]['linkpos'], epos=hrp2krobot.lftarm[i]['linkend'],
                               thickness=20, rgba=lftrgba)
        i = hrp2krobot.lftarm[i]['child']

    # endrotmat = pg.cvtMat4(hrp2krobot.rgtarm[-1]['rotmat'], hrp2krobot.rgtarm[-1]['linkpos'])
    # pg.plotAxisSelf(pandanp, hrp2krobot.rgtarm[-1]['linkend'], endrotmat)
    # endrotmat = pg.cvtMat4(hrp2krobot.lftarm[-1]['rotmat'], hrp2krobot.lftarm[-1]['linkpos'])
    # pg.plotAxisSelf(pandanp, hrp2krobot.lftarm[-1]['linkend'], endrotmat)

def genmnp(hrp2krobot, handpkg, jawwidthrgt = None, jawwidthlft = None):
    """
    generate a panda3d nodepath for the hrp2krobot
    mnp indicates this function generates a mesh nodepath

    :param hrp2krobot: the Hrp2KRobot object, see hrp2k.py
    :return: a nodepath which is ready to be plotted using plotmesh

    author: weiwei
    date: 20170629
    """

    hrp2kmnp = NodePath("hrp2kmnp")

    this_dir, this_filename = os.path.split(__file__)

    # chest0-2, head1 (neck is not plotted)
    hrp2kbody_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp2kegg", "body.egg"))
    hrp2kchest0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp2kegg", "chest0.egg"))
    hrp2kchest1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp2kegg", "chest1.egg"))
    hrp2khead0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp2kegg", "head0.egg"))
    hrp2khead1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp2kegg", "head1.egg"))

    hrp2kbody_model  = loader.loadModel(hrp2kbody_filepath)
    hrp2kchest0_model  = loader.loadModel(hrp2kchest0_filepath)
    hrp2kchest1_model = loader.loadModel(hrp2kchest1_filepath)
    hrp2khead0_model = loader.loadModel(hrp2khead0_filepath)
    hrp2khead1_model = loader.loadModel(hrp2khead1_filepath)

    hrp2kbody_nodepath = NodePath("hrp2kbody")
    hrp2kchest0_nodepath = NodePath("hrp2kchest0")
    hrp2kchest1_nodepath = NodePath("hrp2kchest1")
    hrp2khead0_nodepath = NodePath("hrp2khead0")
    hrp2khead1_nodepath = NodePath("hrp2khead1")

    # body
    hrp2kbody_model.instanceTo(hrp2kbody_nodepath)
    hrp2kbody_rotmat = Mat4.identMat()
    hrp2kbody_nodepath.setMat(hrp2kbody_rotmat)
    hrp2kbody_nodepath.setPos(Vec3(-32, 0, -349.2))
    hrp2kbody_nodepath.reparentTo(hrp2kmnp)
    # chest
    hrp2kchest0_model.instanceTo(hrp2kchest0_nodepath)
    hrp2kchest0_rotmat = pg.cvtMat4(hrp2krobot.base['rotmat'])
    hrp2kchest0_nodepath.setMat(hrp2kchest0_rotmat)
    hrp2kchest0_nodepath.reparentTo(hrp2kmnp)
    hrp2kchest1_model.instanceTo(hrp2kchest1_nodepath)
    hrp2kchest1_rotmat = Mat4.identMat()
    hrp2kchest1_nodepath.setMat(hrp2kchest1_rotmat)
    hrp2kchest1_nodepath.reparentTo(hrp2kchest0_nodepath)
    hrp2khead0_model.instanceTo(hrp2khead0_nodepath)
    hrp2khead0_nodepath.setH(hrp2krobot.initjnts[0])
    hrp2khead0_nodepath.reparentTo(hrp2kchest1_nodepath)
    hrp2khead0_nodepath.setX(-7)
    hrp2khead0_nodepath.setZ(297.3)
    hrp2khead1_model.instanceTo(hrp2khead1_nodepath)
    hrp2khead1_nodepath.setP(hrp2krobot.initjnts[1])
    hrp2khead1_nodepath.reparentTo(hrp2khead0_nodepath)

    # rgtarm
    hrp2krgtarmlj0_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "rarm0.egg"))
    hrp2krgtarmlj0_model = loader.loadModel(hrp2krgtarmlj0_filepath)
    hrp2krgtarmlj0_nodepath = NodePath("hrp2krgtarmlj0_nodepath")
    hrp2krgtarmlj0_model.instanceTo(hrp2krgtarmlj0_nodepath)
    hrp2krgtarmlj0_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[1]['rotmat'], hrp2krobot.rgtarm[1]['linkpos'])
    hrp2krgtarmlj0_nodepath.setMat(hrp2krgtarmlj0_rotmat)
    hrp2krgtarmlj0_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2krgtarmlj1_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "rarm1.egg"))
    hrp2krgtarmlj1_model = loader.loadModel(hrp2krgtarmlj1_filepath)
    hrp2krgtarmlj1_nodepath = NodePath("hrp2krgtarmlj1_nodepath")
    hrp2krgtarmlj1_model.instanceTo(hrp2krgtarmlj1_nodepath)
    hrp2krgtarmlj1_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[2]['rotmat'], hrp2krobot.rgtarm[2]['linkpos'])
    hrp2krgtarmlj1_nodepath.setMat(hrp2krgtarmlj1_rotmat)
    hrp2krgtarmlj1_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2krgtarmlj2_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "rarm2.egg"))
    hrp2krgtarmlj2_model = loader.loadModel(hrp2krgtarmlj2_filepath)
    hrp2krgtarmlj2_nodepath = NodePath("hrp2krgtarmlj2_nodepath")
    hrp2krgtarmlj2_model.instanceTo(hrp2krgtarmlj2_nodepath)
    hrp2krgtarmlj2_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[3]['rotmat'], hrp2krobot.rgtarm[3]['linkpos'])
    hrp2krgtarmlj2_nodepath.setMat(hrp2krgtarmlj2_rotmat)
    hrp2krgtarmlj2_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2krgtarmlj3_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "rarm3.egg"))
    hrp2krgtarmlj3_model = loader.loadModel(hrp2krgtarmlj3_filepath)
    hrp2krgtarmlj3_nodepath = NodePath("hrp2krgtarmlj3_nodepath")
    hrp2krgtarmlj3_model.instanceTo(hrp2krgtarmlj3_nodepath)
    hrp2krgtarmlj3_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[4]['rotmat'], hrp2krobot.rgtarm[4]['linkpos'])
    hrp2krgtarmlj3_nodepath.setMat(hrp2krgtarmlj3_rotmat)
    hrp2krgtarmlj3_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2krgtarmlj4_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "rarm4.egg"))
    hrp2krgtarmlj4_model = loader.loadModel(hrp2krgtarmlj4_filepath)
    hrp2krgtarmlj4_nodepath = NodePath("hrp2krgtarmlj4_nodepath")
    hrp2krgtarmlj4_model.instanceTo(hrp2krgtarmlj4_nodepath)
    hrp2krgtarmlj4_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[5]['rotmat'], hrp2krobot.rgtarm[5]['linkpos'])
    hrp2krgtarmlj4_nodepath.setMat(hrp2krgtarmlj4_rotmat)
    hrp2krgtarmlj4_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2krgtarmlj5_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "rarm5.egg"))
    hrp2krgtarmlj5_model = loader.loadModel(hrp2krgtarmlj5_filepath)
    hrp2krgtarmlj5_nodepath = NodePath("hrp2krgtarmlj5_nodepath")
    hrp2krgtarmlj5_model.instanceTo(hrp2krgtarmlj5_nodepath)
    hrp2krgtarmlj5_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[6]['rotmat'], hrp2krobot.rgtarm[6]['linkpos'])
    hrp2krgtarmlj5_nodepath.setMat(hrp2krgtarmlj5_rotmat)
    hrp2krgtarmlj5_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2krgtarmlj6_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "rarm6.egg"))
    hrp2krgtarmlj6_model = loader.loadModel(hrp2krgtarmlj6_filepath)
    hrp2krgtarmlj6_nodepath = NodePath("hrp2krgtarmlj6_nodepath")
    hrp2krgtarmlj6_model.instanceTo(hrp2krgtarmlj6_nodepath)
    hrp2krgtarmlj6_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[7]['rotmat'], hrp2krobot.rgtarm[7]['linkpos'])
    hrp2krgtarmlj6_nodepath.setMat(hrp2krgtarmlj6_rotmat)
    hrp2krgtarmlj6_nodepath.reparentTo(hrp2kmnp)
    #
    # # lftarm
    hrp2klftarmlj0_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "larm0.egg"))
    hrp2klftarmlj0_model = loader.loadModel(hrp2klftarmlj0_filepath)
    hrp2klftarmlj0_nodepath = NodePath("hrp2klftarmlj0_nodepath")
    hrp2klftarmlj0_model.instanceTo(hrp2klftarmlj0_nodepath)
    hrp2klftarmlj0_rotmat = pg.cvtMat4(hrp2krobot.lftarm[1]['rotmat'], hrp2krobot.lftarm[1]['linkpos'])
    hrp2klftarmlj0_nodepath.setMat(hrp2klftarmlj0_rotmat)
    hrp2klftarmlj0_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2klftarmlj1_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "larm1.egg"))
    hrp2klftarmlj1_model = loader.loadModel(hrp2klftarmlj1_filepath)
    hrp2klftarmlj1_nodepath = NodePath("hrp2klftarmlj1_nodepath")
    hrp2klftarmlj1_model.instanceTo(hrp2klftarmlj1_nodepath)
    hrp2klftarmlj1_rotmat = pg.cvtMat4(hrp2krobot.lftarm[2]['rotmat'], hrp2krobot.lftarm[2]['linkpos'])
    hrp2klftarmlj1_nodepath.setMat(hrp2klftarmlj1_rotmat)
    hrp2klftarmlj1_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2klftarmlj2_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "larm2.egg"))
    hrp2klftarmlj2_model = loader.loadModel(hrp2klftarmlj2_filepath)
    hrp2klftarmlj2_nodepath = NodePath("hrp2klftarmlj2_nodepath")
    hrp2klftarmlj2_model.instanceTo(hrp2klftarmlj2_nodepath)
    hrp2klftarmlj2_rotmat = pg.cvtMat4(hrp2krobot.lftarm[3]['rotmat'], hrp2krobot.lftarm[3]['linkpos'])
    hrp2klftarmlj2_nodepath.setMat(hrp2klftarmlj2_rotmat)
    hrp2klftarmlj2_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2klftarmlj3_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "larm3.egg"))
    hrp2klftarmlj3_model = loader.loadModel(hrp2klftarmlj3_filepath)
    hrp2klftarmlj3_nodepath = NodePath("hrp2klftarmlj3_nodepath")
    hrp2klftarmlj3_model.instanceTo(hrp2klftarmlj3_nodepath)
    hrp2klftarmlj3_rotmat = pg.cvtMat4(hrp2krobot.lftarm[4]['rotmat'], hrp2krobot.lftarm[4]['linkpos'])
    hrp2klftarmlj3_nodepath.setMat(hrp2klftarmlj3_rotmat)
    hrp2klftarmlj3_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2klftarmlj4_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "larm4.egg"))
    hrp2klftarmlj4_model = loader.loadModel(hrp2klftarmlj4_filepath)
    hrp2klftarmlj4_nodepath = NodePath("hrp2klftarmlj4_nodepath")
    hrp2klftarmlj4_model.instanceTo(hrp2klftarmlj4_nodepath)
    hrp2klftarmlj4_rotmat = pg.cvtMat4(hrp2krobot.lftarm[5]['rotmat'], hrp2krobot.lftarm[5]['linkpos'])
    hrp2klftarmlj4_nodepath.setMat(hrp2klftarmlj4_rotmat)
    hrp2klftarmlj4_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2klftarmlj5_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "larm5.egg"))
    hrp2klftarmlj5_model = loader.loadModel(hrp2klftarmlj5_filepath)
    hrp2klftarmlj5_nodepath = NodePath("hrp2klftarmlj5_nodepath")
    hrp2klftarmlj5_model.instanceTo(hrp2klftarmlj5_nodepath)
    hrp2klftarmlj5_rotmat = pg.cvtMat4(hrp2krobot.lftarm[6]['rotmat'], hrp2krobot.lftarm[6]['linkpos'])
    hrp2klftarmlj5_nodepath.setMat(hrp2klftarmlj5_rotmat)
    hrp2klftarmlj5_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2klftarmlj6_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "larm6.egg"))
    hrp2klftarmlj6_model = loader.loadModel(hrp2klftarmlj6_filepath)
    hrp2klftarmlj6_nodepath = NodePath("hrp2klftarmlj6_nodepath")
    hrp2klftarmlj6_model.instanceTo(hrp2klftarmlj6_nodepath)
    hrp2klftarmlj6_rotmat = pg.cvtMat4(hrp2krobot.lftarm[7]['rotmat'], hrp2krobot.lftarm[7]['linkpos'])
    hrp2klftarmlj6_nodepath.setMat(hrp2klftarmlj6_rotmat)
    hrp2klftarmlj6_nodepath.reparentTo(hrp2kmnp)
    #
    # rgthnd
    hrp2krobotrgthnd = handpkg.newHand('rgt')
    hrp2krobotrgtarmlj7_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[7]['rotmat'], hrp2krobot.rgtarm[7]['linkpos'])
    # pg.plotAxisSelf(hrp2kmnp, hrp2krobot.rgtarm[9]['linkend'], hrp2krobotrgtarmlj9_rotmat)
    hrp2krobotrgthnd.reparentTo(hrp2kmnp)
    hrp2krobotrgthnd.setMat(hrp2krobotrgtarmlj7_rotmat)
    pos = hrp2krobotrgthnd.getPos()
    linklength = np.linalg.norm(hrp2krobot.rgtarm[7]['linkvec'])
    hrp2krobotrgthnd.setPos(pos-hrp2krobotrgtarmlj7_rotmat.getRow3(0)*(linklength-135))
    if jawwidthrgt is not None:
        hrp2krobotrgthnd.setJawwidth(jawwidthrgt)
    #
    # lfthnd
    hrp2krobotlfthnd = handpkg.newHand('lft')
    hrp2krobotlftarmlj7_rotmat = pg.cvtMat4(hrp2krobot.lftarm[7]['rotmat'], hrp2krobot.lftarm[7]['linkpos'])
    # pg.plotAxisSelf(hrp2kmnp, hrp2krobot.lftarm[9]['linkend'], hrp2krobotlftarmlj9_rotmat)
    hrp2krobotlfthnd.reparentTo(hrp2kmnp)
    hrp2krobotlfthnd.setMat(hrp2krobotlftarmlj7_rotmat)
    pos = hrp2krobotlfthnd.getPos()
    linklength = np.linalg.norm(hrp2krobot.rgtarm[7]['linkvec'])
    hrp2krobotlfthnd.setPos(pos-hrp2krobotlftarmlj7_rotmat.getRow3(0)*(linklength-135))
    if jawwidthlft is not None:
        hrp2krobotlfthnd.setJawwidth(jawwidthlft)

    return hrp2kmnp
#
#
# def genmnplist(hrp5nrobot, handpkg, jawwidthrgt = None, jawwidthlft = None):
#     """
#     generate a list of panda3d nodepath for the hrp5robot
#     mnp indicates this function generates a mesh nodepath
#
#     return [[right arm mnp list], [leftarm mnp list], [body mnp list]]
#     where one arm mnp list is ordered from base to end-effector
#
#     :param hrp5robot: the Hrp5Robot object, see hrp5.py
#     :return: a nodepath which is ready to be plotted using plotmesh
#
#     author: weiwei
#     date: 20170608
#     """
#
#     hrp5nmnp = NodePath("hrp5nmnp")
#
#     this_dir, this_filename = os.path.split(__file__)
#
#     # chest0-2, head1 (neck is not plotted)
#     hrp5nbody_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Body.egg"))
#     hrp5nchest0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Chest_Link0.egg"))
#     hrp5nchest1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Chest_Link1.egg"))
#     hrp5nchest2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Chest_Link2.egg"))
#     hrp5nhead0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Head_Link0.egg"))
#     hrp5nhead1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Head_Link1.egg"))
#
#     hrp5nbody_model  = loader.loadModel(hrp5nbody_filepath)
#     hrp5nchest0_model  = loader.loadModel(hrp5nchest0_filepath)
#     hrp5nchest1_model = loader.loadModel(hrp5nchest1_filepath)
#     hrp5nchest2_model = loader.loadModel(hrp5nchest2_filepath)
#     hrp5nhead0_model = loader.loadModel(hrp5nhead0_filepath)
#     hrp5nhead1_model = loader.loadModel(hrp5nhead1_filepath)
#
#     hrp5nbody_nodepath = NodePath("hrp5nbody")
#     hrp5nchest0_nodepath = NodePath("hrp5nchest0")
#     hrp5nchest1_nodepath = NodePath("hrp5nchest1")
#     hrp5nchest2_nodepath = NodePath("hrp5nchest2")
#     hrp5nhead0_nodepath = NodePath("hrp5nhead0")
#     hrp5nhead1_nodepath = NodePath("hrp5nhead1")
#
#     # body
#     hrp5nbody_model.instanceTo(hrp5nbody_nodepath)
#     hrp5nbody_rotmat = Mat4.identMat()
#     hrp5nbody_nodepath.setMat(hrp5nbody_rotmat)
#     hrp5nbody_nodepath.setZ(0)
#     # chest
#     hrp5nchest0_model.instanceTo(hrp5nchest0_nodepath)
#     hrp5nchest0_rotmat = Mat4.identMat()
#     hrp5nchest0_nodepath.setMat(hrp5nchest0_rotmat)
#     hrp5nchest0_nodepath.setZ(274)
#     hrp5nchest1_model.instanceTo(hrp5nchest1_nodepath)
#     hrp5nchest1_nodepath.setColor(.7,.7,.2,1)
#     hrp5nchest2_model.instanceTo(hrp5nchest2_nodepath)
#     hrp5nchest2_rotmat = pg.cvtMat4(hrp5nrobot.base['rotmat'])
#     hrp5nchest2_nodepath.setMat(hrp5nchest2_rotmat)
#     hrp5nchest2_nodepath.setColor(.7,.7,.7,1)
#     hrp5nhead0_model.instanceTo(hrp5nhead0_nodepath)
#     hrp5nhead0_nodepath.setH(hrp5nrobot.initjnts[0])
#     hrp5nhead0_nodepath.setX(32)
#     hrp5nhead0_nodepath.setZ(521)
#     hrp5nhead1_model.instanceTo(hrp5nhead1_nodepath)
#     hrp5nhead1_nodepath.setP(hrp5nrobot.initjnts[1])
#     hrp5nhead1_nodepath.setX(40)
#     hrp5nhead1_nodepath.setColor(.5,.5,.5,1)
#
#     hrp5nbody_nodepath.reparentTo(hrp5nmnp)
#     hrp5nchest0_nodepath.reparentTo(hrp5nbody_nodepath)
#     hrp5nchest1_nodepath.reparentTo(hrp5nchest0_nodepath)
#     hrp5nchest2_nodepath.reparentTo(hrp5nchest1_nodepath)
#     hrp5nhead0_nodepath.reparentTo(hrp5nchest2_nodepath)
#     hrp5nhead1_nodepath.reparentTo(hrp5nhead0_nodepath)
#
#     # rgtarm
#     hrp5nrgtarmlj0_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rscap_Link0.egg"))
#     hrp5nrgtarmlj0_model = loader.loadModel(hrp5nrgtarmlj0_filepath)
#     hrp5nrgtarmlj0_nodepath = NodePath("nxtrgtarmlj0_nodepath")
#     hrp5nrgtarmlj0_model.instanceTo(hrp5nrgtarmlj0_nodepath)
#     hrp5nrgtarmlj0_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[1]['rotmat'], hrp5nrobot.rgtarm[1]['linkpos'])
#     hrp5nrgtarmlj0_nodepath.setMat(hrp5nrgtarmlj0_rotmat)
#     hrp5nrgtarmlj0_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nrgtarmlj0_nodepath.reparentTo(hrp5nmnp)
#     #
#     hrp5nrgtarmlj1_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link0.egg"))
#     hrp5nrgtarmlj1_model = loader.loadModel(hrp5nrgtarmlj1_filepath)
#     hrp5nrgtarmlj1_nodepath = NodePath("nxtrgtarmlj1_nodepath")
#     hrp5nrgtarmlj1_model.instanceTo(hrp5nrgtarmlj1_nodepath)
#     hrp5nrgtarmlj1_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[2]['rotmat'], hrp5nrobot.rgtarm[2]['linkpos'])
#     hrp5nrgtarmlj1_nodepath.setMat(hrp5nrgtarmlj1_rotmat)
#     hrp5nrgtarmlj1_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nrgtarmlj1_nodepath.reparentTo(hrp5nmnp)
#     #
#     hrp5nrgtarmlj2_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link1.egg"))
#     hrp5nrgtarmlj2_model = loader.loadModel(hrp5nrgtarmlj2_filepath)
#     hrp5nrgtarmlj2_nodepath = NodePath("nxtrgtarmlj2_nodepath")
#     hrp5nrgtarmlj2_model.instanceTo(hrp5nrgtarmlj2_nodepath)
#     hrp5nrgtarmlj2_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[3]['rotmat'], hrp5nrobot.rgtarm[3]['linkpos'])
#     hrp5nrgtarmlj2_nodepath.setMat(hrp5nrgtarmlj2_rotmat)
#     hrp5nrgtarmlj2_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nrgtarmlj2_nodepath.reparentTo(hrp5nmnp)
#     #
#     hrp5nrgtarmlj3_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link2.egg"))
#     hrp5nrgtarmlj3_model = loader.loadModel(hrp5nrgtarmlj3_filepath)
#     hrp5nrgtarmlj3_nodepath = NodePath("nxtrgtarmlj3_nodepath")
#     hrp5nrgtarmlj3_model.instanceTo(hrp5nrgtarmlj3_nodepath)
#     hrp5nrgtarmlj3_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[4]['rotmat'], hrp5nrobot.rgtarm[4]['linkpos'])
#     hrp5nrgtarmlj3_nodepath.setMat(hrp5nrgtarmlj3_rotmat)
#     hrp5nrgtarmlj3_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nrgtarmlj3_nodepath.reparentTo(hrp5nmnp)
#     #
#     hrp5nrgtarmlj4_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link3.egg"))
#     hrp5nrgtarmlj4_model = loader.loadModel(hrp5nrgtarmlj4_filepath)
#     hrp5nrgtarmlj4_nodepath = NodePath("nxtrgtarmlj4_nodepath")
#     hrp5nrgtarmlj4_model.instanceTo(hrp5nrgtarmlj4_nodepath)
#     hrp5nrgtarmlj4_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[5]['rotmat'], hrp5nrobot.rgtarm[5]['linkpos'])
#     hrp5nrgtarmlj4_nodepath.setMat(hrp5nrgtarmlj4_rotmat)
#     hrp5nrgtarmlj4_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nrgtarmlj4_nodepath.reparentTo(hrp5nmnp)
#     #
#     hrp5nrgtarmlj5_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link4.egg"))
#     hrp5nrgtarmlj5_model = loader.loadModel(hrp5nrgtarmlj5_filepath)
#     hrp5nrgtarmlj5_nodepath = NodePath("nxtrgtarmlj5_nodepath")
#     hrp5nrgtarmlj5_model.instanceTo(hrp5nrgtarmlj5_nodepath)
#     hrp5nrgtarmlj5_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[6]['rotmat'], hrp5nrobot.rgtarm[6]['linkpos'])
#     hrp5nrgtarmlj5_nodepath.setMat(hrp5nrgtarmlj5_rotmat)
#     hrp5nrgtarmlj5_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nrgtarmlj5_nodepath.reparentTo(hrp5nmnp)
#     #
#     hrp5nrgtarmlj6_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link5.egg"))
#     hrp5nrgtarmlj6_model = loader.loadModel(hrp5nrgtarmlj6_filepath)
#     hrp5nrgtarmlj6_nodepath = NodePath("nxtrgtarmlj6_nodepath")
#     hrp5nrgtarmlj6_model.instanceTo(hrp5nrgtarmlj6_nodepath)
#     hrp5nrgtarmlj6_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[7]['rotmat'], hrp5nrobot.rgtarm[7]['linkpos'])
#     hrp5nrgtarmlj6_nodepath.setMat(hrp5nrgtarmlj6_rotmat)
#     hrp5nrgtarmlj6_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nrgtarmlj6_nodepath.reparentTo(hrp5nmnp)
#     #
#     hrp5nrgtarmlj7_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Rarm_Link6.egg"))
#     hrp5nrgtarmlj7_model = loader.loadModel(hrp5nrgtarmlj7_filepath)
#     hrp5nrgtarmlj7_nodepath = NodePath("nxtrgtarmlj7_nodepath")
#     hrp5nrgtarmlj7_model.instanceTo(hrp5nrgtarmlj7_nodepath)
#     hrp5nrgtarmlj7_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[8]['rotmat'], hrp5nrobot.rgtarm[8]['linkpos'])
#     hrp5nrgtarmlj7_nodepath.setMat(hrp5nrgtarmlj7_rotmat)
#     hrp5nrgtarmlj7_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nrgtarmlj7_nodepath.reparentTo(hrp5nmnp)
#
#     # lftarm
#     hrp5nlftarmlj0_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Lscap_Link0.egg"))
#     hrp5nlftarmlj0_model = loader.loadModel(hrp5nlftarmlj0_filepath)
#     hrp5nlftarmlj0_nodepath = NodePath("nxtlftarmlj0_nodepath")
#     hrp5nlftarmlj0_model.instanceTo(hrp5nlftarmlj0_nodepath)
#     hrp5nlftarmlj0_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[1]['rotmat'], hrp5nrobot.lftarm[1]['linkpos'])
#     hrp5nlftarmlj0_nodepath.setMat(hrp5nlftarmlj0_rotmat)
#     hrp5nlftarmlj0_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nlftarmlj0_nodepath.reparentTo(hrp5nmnp)
#     #
#     hrp5nlftarmlj1_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link0.egg"))
#     hrp5nlftarmlj1_model = loader.loadModel(hrp5nlftarmlj1_filepath)
#     hrp5nlftarmlj1_nodepath = NodePath("nxtlftarmlj1_nodepath")
#     hrp5nlftarmlj1_model.instanceTo(hrp5nlftarmlj1_nodepath)
#     hrp5nlftarmlj1_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[2]['rotmat'], hrp5nrobot.lftarm[2]['linkpos'])
#     hrp5nlftarmlj1_nodepath.setMat(hrp5nlftarmlj1_rotmat)
#     hrp5nlftarmlj1_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nlftarmlj1_nodepath.reparentTo(hrp5nmnp)
#     #
#     hrp5nlftarmlj2_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link1.egg"))
#     hrp5nlftarmlj2_model = loader.loadModel(hrp5nlftarmlj2_filepath)
#     hrp5nlftarmlj2_nodepath = NodePath("nxtlftarmlj2_nodepath")
#     hrp5nlftarmlj2_model.instanceTo(hrp5nlftarmlj2_nodepath)
#     hrp5nlftarmlj2_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[3]['rotmat'], hrp5nrobot.lftarm[3]['linkpos'])
#     hrp5nlftarmlj2_nodepath.setMat(hrp5nlftarmlj2_rotmat)
#     hrp5nlftarmlj2_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nlftarmlj2_nodepath.reparentTo(hrp5nmnp)
#     #
#     hrp5nlftarmlj3_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link2.egg"))
#     hrp5nlftarmlj3_model = loader.loadModel(hrp5nlftarmlj3_filepath)
#     hrp5nlftarmlj3_nodepath = NodePath("nxtlftarmlj3_nodepath")
#     hrp5nlftarmlj3_model.instanceTo(hrp5nlftarmlj3_nodepath)
#     hrp5nlftarmlj3_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[4]['rotmat'], hrp5nrobot.lftarm[4]['linkpos'])
#     hrp5nlftarmlj3_nodepath.setMat(hrp5nlftarmlj3_rotmat)
#     hrp5nlftarmlj3_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nlftarmlj3_nodepath.reparentTo(hrp5nmnp)
#     #
#     hrp5nlftarmlj4_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link3.egg"))
#     hrp5nlftarmlj4_model = loader.loadModel(hrp5nlftarmlj4_filepath)
#     hrp5nlftarmlj4_nodepath = NodePath("nxtlftarmlj4_nodepath")
#     hrp5nlftarmlj4_model.instanceTo(hrp5nlftarmlj4_nodepath)
#     hrp5nlftarmlj4_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[5]['rotmat'], hrp5nrobot.lftarm[5]['linkpos'])
#     hrp5nlftarmlj4_nodepath.setMat(hrp5nlftarmlj4_rotmat)
#     hrp5nlftarmlj4_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nlftarmlj4_nodepath.reparentTo(hrp5nmnp)
#     #
#     hrp5nlftarmlj5_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link4.egg"))
#     hrp5nlftarmlj5_model = loader.loadModel(hrp5nlftarmlj5_filepath)
#     hrp5nlftarmlj5_nodepath = NodePath("nxtlftarmlj5_nodepath")
#     hrp5nlftarmlj5_model.instanceTo(hrp5nlftarmlj5_nodepath)
#     hrp5nlftarmlj5_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[6]['rotmat'], hrp5nrobot.lftarm[6]['linkpos'])
#     hrp5nlftarmlj5_nodepath.setMat(hrp5nlftarmlj5_rotmat)
#     hrp5nlftarmlj5_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nlftarmlj5_nodepath.reparentTo(hrp5nmnp)
#     #
#     hrp5nlftarmlj6_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link5.egg"))
#     hrp5nlftarmlj6_model = loader.loadModel(hrp5nlftarmlj6_filepath)
#     hrp5nlftarmlj6_nodepath = NodePath("nxtlftarmlj6_nodepath")
#     hrp5nlftarmlj6_model.instanceTo(hrp5nlftarmlj6_nodepath)
#     hrp5nlftarmlj6_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[7]['rotmat'], hrp5nrobot.lftarm[7]['linkpos'])
#     hrp5nlftarmlj6_nodepath.setMat(hrp5nlftarmlj6_rotmat)
#     hrp5nlftarmlj6_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nlftarmlj6_nodepath.reparentTo(hrp5nmnp)
#     #
#     hrp5nlftarmlj7_filepath = Filename.fromOsSpecific(
#         os.path.join(this_dir, "hrp5negg", "HRP-5P_ConceptDesign_Larm_Link6.egg"))
#     hrp5nlftarmlj7_model = loader.loadModel(hrp5nlftarmlj7_filepath)
#     hrp5nlftarmlj7_nodepath = NodePath("nxtlftarmlj7_nodepath")
#     hrp5nlftarmlj7_model.instanceTo(hrp5nlftarmlj7_nodepath)
#     hrp5nlftarmlj7_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[8]['rotmat'], hrp5nrobot.lftarm[8]['linkpos'])
#     hrp5nlftarmlj7_nodepath.setMat(hrp5nlftarmlj7_rotmat)
#     hrp5nlftarmlj7_nodepath.setColor(.5, .5, .5, 1)
#     hrp5nlftarmlj7_nodepath.reparentTo(hrp5nmnp)
#
#     # rgthnd
#     hrp5nrobotrgthnd = handpkg.newHand('rgt')
#     hrp5nrobotrgtarmlj9_rotmat = pg.cvtMat4(hrp5nrobot.rgtarm[9]['rotmat'], hrp5nrobot.rgtarm[9]['linkpos'])
#     # pg.plotAxisSelf(hrp5nmnp, hrp5nrobot.rgtarm[9]['linkend'], hrp5nrobotrgtarmlj9_rotmat)
#     hrp5nrobotrgthnd.setMat(hrp5nrobotrgtarmlj9_rotmat)
#     hrp5nrobotrgthnd.reparentTo(hrp5nmnp)
#     if jawwidthrgt is not None:
#         hrp5nrobotrgthnd.setJawwidth(jawwidthrgt)
#
#     # lfthnd
#     hrp5nrobotlfthnd = handpkg.newHand('lft')
#     hrp5nrobotlftarmlj9_rotmat = pg.cvtMat4(hrp5nrobot.lftarm[9]['rotmat'], hrp5nrobot.lftarm[9]['linkpos'])
#     # pg.plotAxisSelf(hrp5nmnp, hrp5nrobot.lftarm[9]['linkend'], hrp5nrobotlftarmlj9_rotmat)
#     hrp5nrobotlfthnd.setMat(hrp5nrobotlftarmlj9_rotmat)
#     hrp5nrobotlfthnd.reparentTo(hrp5nmnp)
#     if jawwidthlft is not None:
#         hrp5nrobotlfthnd.setJawwidth(jawwidthlft)
#
#     return [[hrp5nrgtarmlj0_nodepath, hrp5nrgtarmlj1_nodepath, hrp5nrgtarmlj2_nodepath,
#              hrp5nrgtarmlj3_nodepath, hrp5nrgtarmlj4_nodepath, hrp5nrgtarmlj5_nodepath,
#              hrp5nrgtarmlj6_nodepath, hrp5nrgtarmlj7_nodepath, hrp5nrobotrgthnd.handnp],
#             [hrp5nlftarmlj0_nodepath, hrp5nlftarmlj1_nodepath, hrp5nlftarmlj2_nodepath,
#              hrp5nlftarmlj3_nodepath, hrp5nlftarmlj4_nodepath, hrp5nlftarmlj5_nodepath,
#              hrp5nlftarmlj6_nodepath, hrp5nlftarmlj7_nodepath, hrp5nrobotlfthnd.handnp],
#             [hrp5nbody_nodepath]]
#
#
def genmnp_nm(hrp2krobot, handpkg, jawwidthrgt = None, jawwidthlft = None):
    """
    generate a panda3d nodepath for the hrp2krobot
    mnp indicates this function generates a mesh nodepath

    :param hrp2krobot: the Hrp5Robot object, see hrp5.py
    :return: a nodepath which is ready to be plotted using plotmesh

    author: weiwei
    date: 20170630
    """

    hrp2kmnp = NodePath("hrp2kmnp")

    this_dir, this_filename = os.path.split(__file__)

    # chest0-2, head1 (neck is not plotted)
    hrp2kbody_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp2kegg", "body_nm.egg"))
    hrp2kchest0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp2kegg", "chest0_nm.egg"))
    hrp2kchest1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp2kegg", "chest1_nm.egg"))
    hrp2khead0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp2kegg", "head0_nm.egg"))
    hrp2khead1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp2kegg", "head1_nm.egg"))

    hrp2kbody_model  = loader.loadModel(hrp2kbody_filepath)
    hrp2kchest0_model  = loader.loadModel(hrp2kchest0_filepath)
    hrp2kchest1_model = loader.loadModel(hrp2kchest1_filepath)
    hrp2khead0_model = loader.loadModel(hrp2khead0_filepath)
    hrp2khead1_model = loader.loadModel(hrp2khead1_filepath)

    hrp2kbody_nodepath = NodePath("hrp2kbody")
    hrp2kchest0_nodepath = NodePath("hrp2kchest0")
    hrp2kchest1_nodepath = NodePath("hrp2kchest1")
    hrp2khead0_nodepath = NodePath("hrp2khead0")
    hrp2khead1_nodepath = NodePath("hrp2khead1")

    # body
    hrp2kbody_model.instanceTo(hrp2kbody_nodepath)
    hrp2kbody_rotmat = Mat4.identMat()
    hrp2kbody_nodepath.setMat(hrp2kbody_rotmat)
    hrp2kbody_nodepath.setPos(Vec3(-32, 0, -349.2))
    hrp2kbody_nodepath.reparentTo(hrp2kmnp)
    # chest
    hrp2kchest0_model.instanceTo(hrp2kchest0_nodepath)
    hrp2kchest0_rotmat = pg.cvtMat4(hrp2krobot.base['rotmat'])
    hrp2kchest0_nodepath.setMat(hrp2kchest0_rotmat)
    hrp2kchest0_nodepath.reparentTo(hrp2kmnp)
    hrp2kchest1_model.instanceTo(hrp2kchest1_nodepath)
    hrp2kchest1_rotmat = Mat4.identMat()
    hrp2kchest1_nodepath.setMat(hrp2kchest1_rotmat)
    hrp2kchest1_nodepath.reparentTo(hrp2kchest0_nodepath)
    hrp2khead0_model.instanceTo(hrp2khead0_nodepath)
    hrp2khead0_nodepath.setH(hrp2krobot.initjnts[0])
    hrp2khead0_nodepath.reparentTo(hrp2kchest1_nodepath)
    hrp2khead0_nodepath.setX(-7)
    hrp2khead0_nodepath.setZ(297.3)
    hrp2khead1_model.instanceTo(hrp2khead1_nodepath)
    hrp2khead1_nodepath.setP(hrp2krobot.initjnts[1])
    hrp2khead1_nodepath.reparentTo(hrp2khead0_nodepath)

    # rgtarm
    hrp2krgtarmlj0_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "rarm0_nm.egg"))
    hrp2krgtarmlj0_model = loader.loadModel(hrp2krgtarmlj0_filepath)
    hrp2krgtarmlj0_nodepath = NodePath("hrp2krgtarmlj0_nodepath")
    hrp2krgtarmlj0_model.instanceTo(hrp2krgtarmlj0_nodepath)
    hrp2krgtarmlj0_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[1]['rotmat'], hrp2krobot.rgtarm[1]['linkpos'])
    hrp2krgtarmlj0_nodepath.setMat(hrp2krgtarmlj0_rotmat)
    hrp2krgtarmlj0_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2krgtarmlj1_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "rarm1_nm.egg"))
    hrp2krgtarmlj1_model = loader.loadModel(hrp2krgtarmlj1_filepath)
    hrp2krgtarmlj1_nodepath = NodePath("hrp2krgtarmlj1_nodepath")
    hrp2krgtarmlj1_model.instanceTo(hrp2krgtarmlj1_nodepath)
    hrp2krgtarmlj1_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[2]['rotmat'], hrp2krobot.rgtarm[2]['linkpos'])
    hrp2krgtarmlj1_nodepath.setMat(hrp2krgtarmlj1_rotmat)
    hrp2krgtarmlj1_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2krgtarmlj2_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "rarm2_nm.egg"))
    hrp2krgtarmlj2_model = loader.loadModel(hrp2krgtarmlj2_filepath)
    hrp2krgtarmlj2_nodepath = NodePath("hrp2krgtarmlj2_nodepath")
    hrp2krgtarmlj2_model.instanceTo(hrp2krgtarmlj2_nodepath)
    hrp2krgtarmlj2_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[3]['rotmat'], hrp2krobot.rgtarm[3]['linkpos'])
    hrp2krgtarmlj2_nodepath.setMat(hrp2krgtarmlj2_rotmat)
    hrp2krgtarmlj2_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2krgtarmlj3_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "rarm3_nm.egg"))
    hrp2krgtarmlj3_model = loader.loadModel(hrp2krgtarmlj3_filepath)
    hrp2krgtarmlj3_nodepath = NodePath("hrp2krgtarmlj3_nodepath")
    hrp2krgtarmlj3_model.instanceTo(hrp2krgtarmlj3_nodepath)
    hrp2krgtarmlj3_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[4]['rotmat'], hrp2krobot.rgtarm[4]['linkpos'])
    hrp2krgtarmlj3_nodepath.setMat(hrp2krgtarmlj3_rotmat)
    hrp2krgtarmlj3_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2krgtarmlj4_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "rarm4_nm.egg"))
    hrp2krgtarmlj4_model = loader.loadModel(hrp2krgtarmlj4_filepath)
    hrp2krgtarmlj4_nodepath = NodePath("hrp2krgtarmlj4_nodepath")
    hrp2krgtarmlj4_model.instanceTo(hrp2krgtarmlj4_nodepath)
    hrp2krgtarmlj4_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[5]['rotmat'], hrp2krobot.rgtarm[5]['linkpos'])
    hrp2krgtarmlj4_nodepath.setMat(hrp2krgtarmlj4_rotmat)
    hrp2krgtarmlj4_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2krgtarmlj5_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "rarm5_nm.egg"))
    hrp2krgtarmlj5_model = loader.loadModel(hrp2krgtarmlj5_filepath)
    hrp2krgtarmlj5_nodepath = NodePath("hrp2krgtarmlj5_nodepath")
    hrp2krgtarmlj5_model.instanceTo(hrp2krgtarmlj5_nodepath)
    hrp2krgtarmlj5_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[6]['rotmat'], hrp2krobot.rgtarm[6]['linkpos'])
    hrp2krgtarmlj5_nodepath.setMat(hrp2krgtarmlj5_rotmat)
    hrp2krgtarmlj5_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2krgtarmlj6_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "rarm6_nm.egg"))
    hrp2krgtarmlj6_model = loader.loadModel(hrp2krgtarmlj6_filepath)
    hrp2krgtarmlj6_nodepath = NodePath("hrp2krgtarmlj6_nodepath")
    hrp2krgtarmlj6_model.instanceTo(hrp2krgtarmlj6_nodepath)
    hrp2krgtarmlj6_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[7]['rotmat'], hrp2krobot.rgtarm[7]['linkpos'])
    hrp2krgtarmlj6_nodepath.setMat(hrp2krgtarmlj6_rotmat)
    hrp2krgtarmlj6_nodepath.reparentTo(hrp2kmnp)
    #
    # # lftarm
    hrp2klftarmlj0_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "larm0_nm.egg"))
    hrp2klftarmlj0_model = loader.loadModel(hrp2klftarmlj0_filepath)
    hrp2klftarmlj0_nodepath = NodePath("hrp2klftarmlj0_nodepath")
    hrp2klftarmlj0_model.instanceTo(hrp2klftarmlj0_nodepath)
    hrp2klftarmlj0_rotmat = pg.cvtMat4(hrp2krobot.lftarm[1]['rotmat'], hrp2krobot.lftarm[1]['linkpos'])
    hrp2klftarmlj0_nodepath.setMat(hrp2klftarmlj0_rotmat)
    hrp2klftarmlj0_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2klftarmlj1_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "larm1_nm.egg"))
    hrp2klftarmlj1_model = loader.loadModel(hrp2klftarmlj1_filepath)
    hrp2klftarmlj1_nodepath = NodePath("hrp2klftarmlj1_nodepath")
    hrp2klftarmlj1_model.instanceTo(hrp2klftarmlj1_nodepath)
    hrp2klftarmlj1_rotmat = pg.cvtMat4(hrp2krobot.lftarm[2]['rotmat'], hrp2krobot.lftarm[2]['linkpos'])
    hrp2klftarmlj1_nodepath.setMat(hrp2klftarmlj1_rotmat)
    hrp2klftarmlj1_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2klftarmlj2_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "larm2_nm.egg"))
    hrp2klftarmlj2_model = loader.loadModel(hrp2klftarmlj2_filepath)
    hrp2klftarmlj2_nodepath = NodePath("hrp2klftarmlj2_nodepath")
    hrp2klftarmlj2_model.instanceTo(hrp2klftarmlj2_nodepath)
    hrp2klftarmlj2_rotmat = pg.cvtMat4(hrp2krobot.lftarm[3]['rotmat'], hrp2krobot.lftarm[3]['linkpos'])
    hrp2klftarmlj2_nodepath.setMat(hrp2klftarmlj2_rotmat)
    hrp2klftarmlj2_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2klftarmlj3_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "larm3_nm.egg"))
    hrp2klftarmlj3_model = loader.loadModel(hrp2klftarmlj3_filepath)
    hrp2klftarmlj3_nodepath = NodePath("hrp2klftarmlj3_nodepath")
    hrp2klftarmlj3_model.instanceTo(hrp2klftarmlj3_nodepath)
    hrp2klftarmlj3_rotmat = pg.cvtMat4(hrp2krobot.lftarm[4]['rotmat'], hrp2krobot.lftarm[4]['linkpos'])
    hrp2klftarmlj3_nodepath.setMat(hrp2klftarmlj3_rotmat)
    hrp2klftarmlj3_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2klftarmlj4_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "larm4_nm.egg"))
    hrp2klftarmlj4_model = loader.loadModel(hrp2klftarmlj4_filepath)
    hrp2klftarmlj4_nodepath = NodePath("hrp2klftarmlj4_nodepath")
    hrp2klftarmlj4_model.instanceTo(hrp2klftarmlj4_nodepath)
    hrp2klftarmlj4_rotmat = pg.cvtMat4(hrp2krobot.lftarm[5]['rotmat'], hrp2krobot.lftarm[5]['linkpos'])
    hrp2klftarmlj4_nodepath.setMat(hrp2klftarmlj4_rotmat)
    hrp2klftarmlj4_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2klftarmlj5_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "larm5_nm.egg"))
    hrp2klftarmlj5_model = loader.loadModel(hrp2klftarmlj5_filepath)
    hrp2klftarmlj5_nodepath = NodePath("hrp2klftarmlj5_nodepath")
    hrp2klftarmlj5_model.instanceTo(hrp2klftarmlj5_nodepath)
    hrp2klftarmlj5_rotmat = pg.cvtMat4(hrp2krobot.lftarm[6]['rotmat'], hrp2krobot.lftarm[6]['linkpos'])
    hrp2klftarmlj5_nodepath.setMat(hrp2klftarmlj5_rotmat)
    hrp2klftarmlj5_nodepath.reparentTo(hrp2kmnp)
    # #
    hrp2klftarmlj6_filepath = Filename.fromOsSpecific(
        os.path.join(this_dir, "hrp2kegg", "larm6_nm.egg"))
    hrp2klftarmlj6_model = loader.loadModel(hrp2klftarmlj6_filepath)
    hrp2klftarmlj6_nodepath = NodePath("hrp2klftarmlj6_nodepath")
    hrp2klftarmlj6_model.instanceTo(hrp2klftarmlj6_nodepath)
    hrp2klftarmlj6_rotmat = pg.cvtMat4(hrp2krobot.lftarm[7]['rotmat'], hrp2krobot.lftarm[7]['linkpos'])
    hrp2klftarmlj6_nodepath.setMat(hrp2klftarmlj6_rotmat)
    hrp2klftarmlj6_nodepath.reparentTo(hrp2kmnp)
    #
    # rgthnd
    hrp2krobotrgthnd = handpkg.newHandNM('rgt')
    hrp2krobotrgtarmlj7_rotmat = pg.cvtMat4(hrp2krobot.rgtarm[7]['rotmat'], hrp2krobot.rgtarm[7]['linkpos'])
    # pg.plotAxisSelf(hrp2kmnp, hrp2krobot.rgtarm[9]['linkend'], hrp2krobotrgtarmlj9_rotmat)
    hrp2krobotrgthnd.reparentTo(hrp2kmnp)
    hrp2krobotrgthnd.setMat(hrp2krobotrgtarmlj7_rotmat)
    pos = hrp2krobotrgthnd.getPos()
    linklength = np.linalg.norm(hrp2krobot.rgtarm[7]['linkvec'])
    hrp2krobotrgthnd.setPos(pos-hrp2krobotrgtarmlj7_rotmat.getRow3(0)*(linklength-135))
    if jawwidthrgt is not None:
        hrp2krobotrgthnd.setJawwidth(jawwidthrgt)
    #
    # lfthnd
    hrp2krobotlfthnd = handpkg.newHandNM('lft')
    hrp2krobotlftarmlj7_rotmat = pg.cvtMat4(hrp2krobot.lftarm[7]['rotmat'], hrp2krobot.lftarm[7]['linkpos'])
    # pg.plotAxisSelf(hrp2kmnp, hrp2krobot.lftarm[9]['linkend'], hrp2krobotlftarmlj9_rotmat)
    hrp2krobotlfthnd.reparentTo(hrp2kmnp)
    hrp2krobotlfthnd.setMat(hrp2krobotlftarmlj7_rotmat)
    pos = hrp2krobotlfthnd.getPos()
    linklength = np.linalg.norm(hrp2krobot.rgtarm[7]['linkvec'])
    hrp2krobotlfthnd.setPos(pos-hrp2krobotlftarmlj7_rotmat.getRow3(0)*(linklength-135))
    if jawwidthlft is not None:
        hrp2krobotlfthnd.setJawwidth(jawwidthlft)

    hrp2kmnp.setTransparency(TransparencyAttrib.MAlpha)

    return hrp2kmnp