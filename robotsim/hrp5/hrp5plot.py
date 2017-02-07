import os

import numpy as np
from manipulation.grip.robotiq85 import rtq85
from panda3d.core import *

import pandaplotutils.pandageom as pg


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


def genHrp5mnp(hrp5robot, jawwidthrgt = None, jawwidthlft = None):
    """
    generate a panda3d nodepath for the hrp5robot
    mnp indicates this function generates a mesh nodepath

    :param hrp5robot: the Hrp5Robot object, see hrp5.py
    :return: a nodepath which is ready to be plotted using plotmesh

    author: weiwei
    date: 20161202
    """

    hrp5mnp = NodePath("hrp5mnp")

    this_dir, this_filename = os.path.split(__file__)

    # chest0-2, head1 (neck is not plotted)
    hrp5chest0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5egg", "hrp5_chest0.egg"))
    hrp5chest1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5egg", "hrp5_chest1.egg"))
    hrp5chest2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5egg", "hrp5_chest2.egg"))
    hrp5head1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5egg", "hrp5_head1.egg"))

    hrp5chest0_model  = loader.loadModel(hrp5chest0_filepath)
    hrp5chest1_model = loader.loadModel(hrp5chest1_filepath)
    hrp5chest2_model = loader.loadModel(hrp5chest2_filepath)
    hrp5head1_model = loader.loadModel(hrp5head1_filepath)

    hrp5chest0_nodepath = NodePath("hrp5chest0")
    hrp5chest1_nodepath = NodePath("hrp5chest1")
    hrp5chest2_nodepath = NodePath("hrp5chest2")
    hrp5head1_nodepath = NodePath("hrp5head1")

    hrp5chest0_model.instanceTo(hrp5chest0_nodepath)
    hrp5chest0_rotmat = pg.cvtMat4(hrp5robot.base['rotmat'])
    hrp5chest0_nodepath.setMat(hrp5chest0_rotmat)
    hrp5chest0_nodepath.setZ(223)
    hrp5chest1_model.instanceTo(hrp5chest1_nodepath)
    hrp5chest1_nodepath.setColor(.7,.7,.2,1)
    hrp5chest2_model.instanceTo(hrp5chest2_nodepath)
    hrp5chest2_nodepath.setX(-68)
    hrp5chest2_nodepath.setColor(.7,.7,.7,1)
    hrp5head1_model.instanceTo(hrp5head1_nodepath)
    hrp5head1_nodepath.setH(hrp5robot.initjnts[0])
    hrp5head1_nodepath.setP(hrp5robot.initjnts[1])
    hrp5head1_nodepath.setX(43)
    hrp5head1_nodepath.setZ(440)
    hrp5head1_nodepath.setColor(.5,.5,.5,1)

    hrp5chest0_nodepath.reparentTo(hrp5mnp)
    hrp5chest1_nodepath.reparentTo(hrp5chest0_nodepath)
    hrp5chest2_nodepath.reparentTo(hrp5chest0_nodepath)
    hrp5head1_nodepath.reparentTo(hrp5chest0_nodepath)

    # rgtarm
    hrp5rgtarmlj0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5egg", "hrp5_rscap.egg"))
    hrp5rgtarmlj0_model = loader.loadModel(hrp5rgtarmlj0_filepath)
    hrp5rgtarmlj0_nodepath = NodePath("nxtrgtarmlj0_nodepath")
    hrp5rgtarmlj0_model.instanceTo(hrp5rgtarmlj0_nodepath)
    hrp5rgtarmlj0_rotmat = pg.cvtMat4(hrp5robot.rgtarm[1]['rotmat'], hrp5robot.rgtarm[1]['linkpos'])
    hrp5rgtarmlj0_nodepath.setMat(hrp5rgtarmlj0_rotmat)
    hrp5rgtarmlj0_nodepath.setColor(.5,.5,.5,1)
    hrp5rgtarmlj0_nodepath.reparentTo(hrp5mnp)
    #
    hrp5rgtarmlj1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5egg", "hrp5_rlink0.egg"))
    hrp5rgtarmlj1_model = loader.loadModel(hrp5rgtarmlj1_filepath)
    hrp5rgtarmlj1_nodepath = NodePath("nxtrgtarmlj1_nodepath")
    hrp5rgtarmlj1_model.instanceTo(hrp5rgtarmlj1_nodepath)
    hrp5rgtarmlj1_rotmat = pg.cvtMat4(hrp5robot.rgtarm[2]['rotmat'], hrp5robot.rgtarm[2]['linkpos'])
    hrp5rgtarmlj1_nodepath.setMat(hrp5rgtarmlj1_rotmat)
    hrp5rgtarmlj1_nodepath.setColor(.5,.5,.5,1)
    hrp5rgtarmlj1_nodepath.reparentTo(hrp5mnp)
    #
    hrp5rgtarmlj2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5egg", "hrp5_rlink1.egg"))
    hrp5rgtarmlj2_model = loader.loadModel(hrp5rgtarmlj2_filepath)
    hrp5rgtarmlj2_nodepath = NodePath("nxtrgtarmlj2_nodepath")
    hrp5rgtarmlj2_model.instanceTo(hrp5rgtarmlj2_nodepath)
    hrp5rgtarmlj2_rotmat = pg.cvtMat4(hrp5robot.rgtarm[3]['rotmat'], hrp5robot.rgtarm[3]['linkpos'])
    hrp5rgtarmlj2_nodepath.setMat(hrp5rgtarmlj2_rotmat)
    hrp5rgtarmlj2_nodepath.setColor(.5,.5,.5,1)
    hrp5rgtarmlj2_nodepath.reparentTo(hrp5mnp)
    #
    hrp5rgtarmlj3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5egg", "hrp5_rlink2.egg"))
    hrp5rgtarmlj3_model = loader.loadModel(hrp5rgtarmlj3_filepath)
    hrp5rgtarmlj3_nodepath = NodePath("nxtrgtarmlj3_nodepath")
    hrp5rgtarmlj3_model.instanceTo(hrp5rgtarmlj3_nodepath)
    hrp5rgtarmlj3_rotmat = pg.cvtMat4(hrp5robot.rgtarm[4]['rotmat'], hrp5robot.rgtarm[4]['linkpos'])
    hrp5rgtarmlj3_nodepath.setMat(hrp5rgtarmlj3_rotmat)
    hrp5rgtarmlj3_nodepath.setColor(.5,.5,.5,1)
    hrp5rgtarmlj3_nodepath.reparentTo(hrp5mnp)
    #
    hrp5rgtarmlj4_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5egg", "hrp5_rlink3.egg"))
    hrp5rgtarmlj4_model = loader.loadModel(hrp5rgtarmlj4_filepath)
    hrp5rgtarmlj4_nodepath = NodePath("nxtrgtarmlj4_nodepath")
    hrp5rgtarmlj4_model.instanceTo(hrp5rgtarmlj4_nodepath)
    hrp5rgtarmlj4_rotmat = pg.cvtMat4(hrp5robot.rgtarm[5]['rotmat'], hrp5robot.rgtarm[5]['linkpos'])
    hrp5rgtarmlj4_nodepath.setMat(hrp5rgtarmlj4_rotmat)
    hrp5rgtarmlj4_nodepath.setColor(.5,.5,.5,1)
    hrp5rgtarmlj4_nodepath.reparentTo(hrp5mnp)
    #
    hrp5rgtarmlj5_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5egg", "hrp5_rlink4.egg"))
    hrp5rgtarmlj5_model = loader.loadModel(hrp5rgtarmlj5_filepath)
    hrp5rgtarmlj5_nodepath = NodePath("nxtrgtarmlj5_nodepath")
    hrp5rgtarmlj5_model.instanceTo(hrp5rgtarmlj5_nodepath)
    hrp5rgtarmlj5_rotmat = pg.cvtMat4(hrp5robot.rgtarm[6]['rotmat'], hrp5robot.rgtarm[6]['linkpos'])
    hrp5rgtarmlj5_nodepath.setMat(hrp5rgtarmlj5_rotmat)
    hrp5rgtarmlj5_nodepath.setColor(.5,.5,.5,1)
    hrp5rgtarmlj5_nodepath.reparentTo(hrp5mnp)
    #
    hrp5rgtarmlj6_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5egg", "hrp5_rlink5.egg"))
    hrp5rgtarmlj6_model = loader.loadModel(hrp5rgtarmlj6_filepath)
    hrp5rgtarmlj6_nodepath = NodePath("nxtrgtarmlj6_nodepath")
    hrp5rgtarmlj6_model.instanceTo(hrp5rgtarmlj6_nodepath)
    hrp5rgtarmlj6_rotmat = pg.cvtMat4(hrp5robot.rgtarm[7]['rotmat'], hrp5robot.rgtarm[7]['linkpos'])
    hrp5rgtarmlj6_nodepath.setMat(hrp5rgtarmlj6_rotmat)
    hrp5rgtarmlj6_nodepath.setColor(.5,.5,.5,1)
    hrp5rgtarmlj6_nodepath.reparentTo(hrp5mnp)
    #
    hrp5rgtarmlj7_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "hrp5egg", "hrp5_rlink6.egg"))
    hrp5rgtarmlj7_model = loader.loadModel(hrp5rgtarmlj7_filepath)
    hrp5rgtarmlj7_nodepath = NodePath("nxtrgtarmlj7_nodepath")
    hrp5rgtarmlj7_model.instanceTo(hrp5rgtarmlj7_nodepath)
    hrp5rgtarmlj7_rotmat = pg.cvtMat4(hrp5robot.rgtarm[8]['rotmat'], hrp5robot.rgtarm[8]['linkpos'])
    hrp5rgtarmlj7_nodepath.setMat(hrp5rgtarmlj7_rotmat)
    hrp5rgtarmlj7_nodepath.setColor(.5,.5,.5,1)
    hrp5rgtarmlj7_nodepath.reparentTo(hrp5mnp)
    #
    # # lftarm
    # nxtlftarmlj0_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj0.egg"))
    # nxtlftarmlj0_model = loader.loadModel(nxtlftarmlj0_filepath)
    # nxtlftarmlj0_nodepath = NodePath("nxtlftarmlj0_nodepath")
    # nxtlftarmlj0_model.instanceTo(nxtlftarmlj0_nodepath)
    # nxtlftarmlj0_rotmat = pg.cvtMat4(nxtrobot.lftarm[1]['rotmat'], nxtrobot.lftarm[1]['linkpos'])
    # nxtlftarmlj0_nodepath.setMat(nxtlftarmlj0_rotmat)
    # nxtlftarmlj0_nodepath.reparentTo(nxtmnp)
    #
    # nxtlftarmlj1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj1.egg"))
    # nxtlftarmlj1_model = loader.loadModel(nxtlftarmlj1_filepath)
    # nxtlftarmlj1_nodepath = NodePath("nxtlftarmlj1_nodepath")
    # nxtlftarmlj1_model.instanceTo(nxtlftarmlj1_nodepath)
    # nxtlftarmlj1_rotmat = pg.cvtMat4(nxtrobot.lftarm[2]['rotmat'], nxtrobot.lftarm[2]['linkpos'])
    # nxtlftarmlj1_nodepath.setMat(nxtlftarmlj1_rotmat)
    # nxtlftarmlj1_nodepath.reparentTo(nxtmnp)
    #
    # nxtlftarmlj2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj2.egg"))
    # nxtlftarmlj2_model = loader.loadModel(nxtlftarmlj2_filepath)
    # nxtlftarmlj2_nodepath = NodePath("nxtlftarmlj2_nodepath")
    # nxtlftarmlj2_model.instanceTo(nxtlftarmlj2_nodepath)
    # nxtlftarmlj2_rotmat = pg.cvtMat4(nxtrobot.lftarm[3]['rotmat'], nxtrobot.lftarm[3]['linkpos'])
    # nxtlftarmlj2_nodepath.setMat(nxtlftarmlj2_rotmat)
    # nxtlftarmlj2_nodepath.reparentTo(nxtmnp)
    #
    # nxtlftarmlj3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj3.egg"))
    # nxtlftarmlj3_model = loader.loadModel(nxtlftarmlj3_filepath)
    # nxtlftarmlj3_nodepath = NodePath("nxtlftarmlj3_nodepath")
    # nxtlftarmlj3_model.instanceTo(nxtlftarmlj3_nodepath)
    # nxtlftarmlj3_rotmat = pg.cvtMat4(nxtrobot.lftarm[4]['rotmat'], nxtrobot.lftarm[4]['linkpos'])
    # nxtlftarmlj3_nodepath.setMat(nxtlftarmlj3_rotmat)
    # nxtlftarmlj3_nodepath.reparentTo(nxtmnp)
    #
    # nxtlftarmlj4_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "nxtegg", "nxt_lftarm_lj4.egg"))
    # nxtlftarmlj4_model = loader.loadModel(nxtlftarmlj4_filepath)
    # nxtlftarmlj4_nodepath = NodePath("nxtlftarmlj4_nodepath")
    # nxtlftarmlj4_model.instanceTo(nxtlftarmlj4_nodepath)
    # nxtlftarmlj4_rotmat = pg.cvtMat4(nxtrobot.lftarm[5]['rotmat'], nxtrobot.lftarm[5]['linkpos'])
    # nxtlftarmlj4_nodepath.setMat(nxtlftarmlj4_rotmat)
    # nxtlftarmlj4_nodepath.reparentTo(nxtmnp)

    # rgthnd
    hrp5robotrgthnd = rtq85.Rtq85()
    hrp5robotrgtarmlj9_rotmat = pg.cvtMat4(hrp5robot.rgtarm[9]['rotmat'], hrp5robot.rgtarm[9]['linkpos'])
    # pg.plotAxisSelf(hrp5mnp, hrp5robot.rgtarm[9]['linkend'], hrp5robotrgtarmlj9_rotmat)
    hrp5robotrgthnd.setMat(hrp5robotrgtarmlj9_rotmat)
    hrp5robotrgthnd.reparentTo(hrp5mnp)
    if jawwidthrgt is not None:
        hrp5robotrgthnd.setJawwidth(jawwidthrgt)

    # lfthnd
    # hrp5robotlfthnd = rtq85.Rtq85()
    # hrp5robotlftarmlj9_rotmat = pg.cvtMat4(hrp5robot.lftarm[9]['rotmat'], hrp5robot.lftarm[9]['linkpos'])
    # pg.plotAxisSelf(hrp5mnp, hrp5robot.lftarm[9]['linkend'], hrp5robotlftarmlj9_rotmat)
    # hrp5robotlfthnd.setMat(hrp5robotlftarmlj9_rotmat)
    # hrp5robotlfthnd.reparentTo(hrp5mnp)
    # if jawwidthlft is not None:
    #     hrp5robotlfthnd.setJawwidth(jawwidthlft)

    return hrp5mnp