import os

import numpy as np
from manipulation.grip.robotiq85 import rtq85
from manipulation.grip.hrp5three import hrp5three
from panda3d.core import *

import pandaplotutils.pandageom as pg

def plotstick(pandanp, ur5robot, rgtrbga=np.array([.5 ,0 ,0 ,0]), lftrgba=np.array([.5 ,0 ,0 ,0])):
    """
    plot the stick model of the ur5robot robot in panda3d

    :param pandanp: a panda3d nodepath
    :param ur5robot:
    :param rgtrbga: color of right arm
    :param lftrgba: color of left arm
    :return: null

    author: weiwei
    date: 20161202
    """

    i = 0
    while i != -1:
        pg.plotDumbbell(pandanp, spos=ur5robot.rgtarm[i]['linkpos'], epos=ur5robot.rgtarm[i]['linkend'],
                               thickness=20, rgba=rgtrbga)
        i = ur5robot.rgtarm[i]['child']
    i = 0
    while i != -1:
        pg.plotDumbbell(pandanp, spos=ur5robot.lftarm[i]['linkpos'], epos=ur5robot.lftarm[i]['linkend'],
                               thickness=20, rgba=lftrgba)
        i = ur5robot.lftarm[i]['child']

def genmnp(ur5dual, handpkg, jawwidthrgt = None, jawwidthlft = None):
    """
    generate a panda3d nodepath for the ur5dual
    mnp indicates this function generates a mesh nodepath

    :param ur5dual:
    :return: a nodepath which is ready to be plotted using plotmesh

    author: weiwei
    date: 20161202
    """

    ur5mnp = NodePath("ur5dualmnp")

    this_dir, this_filename = os.path.split(__file__)

    # chest0-2, head1 (neck is not plotted)
    ur5base_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur5egg", "base.egg"))
    ur5upperarm_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur5egg", "upperarm.egg"))
    ur5shoulder_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur5egg", "shoulder.egg"))
    ur5forearm_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur5egg", "forearm.egg"))
    ur5wrist1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur5egg", "wrist1.egg"))
    ur5wrist2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur5egg", "wrist2.egg"))
    ur5wrist3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur5egg", "wrist3.egg"))

    ur5base_model  = loader.loadModel(ur5base_filepath)
    ur5upperarm_model  = loader.loadModel(ur5upperarm_filepath)
    ur5shoulder_model = loader.loadModel(ur5shoulder_filepath)
    ur5forearm_model = loader.loadModel(ur5forearm_filepath)
    ur5wrist1_model = loader.loadModel(ur5wrist1_filepath)
    ur5wrist2_model = loader.loadModel(ur5wrist2_filepath)
    ur5wrist3_model = loader.loadModel(ur5wrist3_filepath)

    # rgt
    ur5rgtbase_nodepath = NodePath("ur5rgtbase")
    ur5rgtshoulder_nodepath = NodePath("ur5rgtshoulder")
    ur5rgtupperarm_nodepath = NodePath("ur5rgtupperarm")
    ur5rgtforearm_nodepath = NodePath("ur5rgtforearm")
    ur5rgtwrist1_nodepath = NodePath("ur5rgtwrist1")
    ur5rgtwrist2_nodepath = NodePath("ur5rgtwrist2")
    ur5rgtwrist3_nodepath = NodePath("ur5rgtwrist3")
    #
    # base egg coordinates is retracted 89.2 along z to fit parameters
    ur5base_model.instanceTo(ur5rgtbase_nodepath)
    ur5base_rotmat = pg.cvtMat4(ur5dual.rgtarm[1]['inherentR'], ur5dual.rgtarm[1]['linkpos'])
    ur5rgtbase_nodepath.setMat(ur5base_rotmat)
    ur5rgtbase_nodepath.setColor(.5,.5,.5,1)
    ur5rgtbase_nodepath.reparentTo(ur5mnp)
    #
    ur5shoulder_model.instanceTo(ur5rgtshoulder_nodepath)
    ur5shoulder_rotmat = pg.cvtMat4(ur5dual.rgtarm[1]['rotmat'], ur5dual.rgtarm[1]['linkpos'])
    ur5rgtshoulder_nodepath.setMat(ur5shoulder_rotmat)
    ur5rgtshoulder_nodepath.setColor(.5,.7,.3,1)
    ur5rgtshoulder_nodepath.reparentTo(ur5mnp)
    #
    ur5upperarm_model.instanceTo(ur5rgtupperarm_nodepath)
    ur5upperarm_rotmat = pg.cvtMat4(ur5dual.rgtarm[2]['rotmat'], ur5dual.rgtarm[2]['linkpos'])
    ur5rgtupperarm_nodepath.setMat(ur5upperarm_rotmat)
    ur5rgtupperarm_nodepath.setColor(.5,.5,.5,1)
    ur5rgtupperarm_nodepath.reparentTo(ur5mnp)
    #
    ur5forearm_model.instanceTo(ur5rgtforearm_nodepath)
    ur5forearm_rotmat = pg.cvtMat4(ur5dual.rgtarm[3]['rotmat'], ur5dual.rgtarm[3]['linkpos'])
    ur5rgtforearm_nodepath.setMat(ur5forearm_rotmat)
    ur5rgtforearm_nodepath.setColor(.5,.5,.5,1)
    ur5rgtforearm_nodepath.reparentTo(ur5mnp)
    #
    ur5wrist1_model.instanceTo(ur5rgtwrist1_nodepath)
    ur5wrist1_rotmat = pg.cvtMat4(ur5dual.rgtarm[4]['rotmat'], ur5dual.rgtarm[4]['linkpos'])
    ur5rgtwrist1_nodepath.setMat(ur5wrist1_rotmat)
    ur5rgtwrist1_nodepath.setColor(.5,.7,.3,1)
    ur5rgtwrist1_nodepath.reparentTo(ur5mnp)
    #
    ur5wrist2_model.instanceTo(ur5rgtwrist2_nodepath)
    ur5wrist2_rotmat = pg.cvtMat4(ur5dual.rgtarm[5]['rotmat'], ur5dual.rgtarm[5]['linkpos'])
    ur5rgtwrist2_nodepath.setMat(ur5wrist2_rotmat)
    ur5rgtwrist2_nodepath.setColor(.5,.5,.5,1)
    ur5rgtwrist2_nodepath.reparentTo(ur5mnp)
    #
    # wrist3 egg coordinates is rotated 90 around z retracted 72.33752-81.82489 to fit parameters
    ur5wrist3_model.instanceTo(ur5rgtwrist3_nodepath)
    ur5wrist3_rotmat = pg.cvtMat4(ur5dual.rgtarm[6]['rotmat'], ur5dual.rgtarm[6]['linkpos'])
    ur5rgtwrist3_nodepath.setMat(ur5wrist3_rotmat)
    ur5rgtwrist3_nodepath.setColor(.5,.5,.5,1)
    ur5rgtwrist3_nodepath.reparentTo(ur5mnp)

    # lft
    ur5lftbase_nodepath = NodePath("ur5lftbase")
    ur5lftupperarm_nodepath = NodePath("ur5lftupperarm")
    ur5lftshoulder_nodepath = NodePath("ur5lftshoulder")
    ur5lftforearm_nodepath = NodePath("ur5lftforearm")
    ur5lftwrist1_nodepath = NodePath("ur5lftwrist1")
    ur5lftwrist2_nodepath = NodePath("ur5lftwrist2")
    ur5lftwrist3_nodepath = NodePath("ur5lftwrist3")
    #
    # base egg coordinates is retracted 89.2 along z to fit parameters
    ur5base_model.instanceTo(ur5lftbase_nodepath)
    ur5base_rotmat = pg.cvtMat4(ur5dual.lftarm[1]['inherentR'], ur5dual.lftarm[1]['linkpos'])
    ur5lftbase_nodepath.setMat(ur5base_rotmat)
    ur5lftbase_nodepath.setColor(.5,.5,.5,1)
    ur5lftbase_nodepath.reparentTo(ur5mnp)
    #
    ur5shoulder_model.instanceTo(ur5lftshoulder_nodepath)
    ur5shoulder_rotmat = pg.cvtMat4(ur5dual.lftarm[1]['rotmat'], ur5dual.lftarm[1]['linkpos'])
    ur5lftshoulder_nodepath.setMat(ur5shoulder_rotmat)
    ur5lftshoulder_nodepath.setColor(.5,.7,.3,1)
    ur5lftshoulder_nodepath.reparentTo(ur5mnp)
    #
    ur5upperarm_model.instanceTo(ur5lftupperarm_nodepath)
    ur5upperarm_rotmat = pg.cvtMat4(ur5dual.lftarm[2]['rotmat'], ur5dual.lftarm[2]['linkpos'])
    ur5lftupperarm_nodepath.setMat(ur5upperarm_rotmat)
    ur5lftupperarm_nodepath.setColor(.5,.5,.5,1)
    ur5lftupperarm_nodepath.reparentTo(ur5mnp)
    #
    ur5forearm_model.instanceTo(ur5lftforearm_nodepath)
    ur5forearm_rotmat = pg.cvtMat4(ur5dual.lftarm[3]['rotmat'], ur5dual.lftarm[3]['linkpos'])
    ur5lftforearm_nodepath.setMat(ur5forearm_rotmat)
    ur5lftforearm_nodepath.setColor(.5,.5,.5,1)
    ur5lftforearm_nodepath.reparentTo(ur5mnp)
    #
    ur5wrist1_model.instanceTo(ur5lftwrist1_nodepath)
    ur5wrist1_rotmat = pg.cvtMat4(ur5dual.lftarm[4]['rotmat'], ur5dual.lftarm[4]['linkpos'])
    ur5lftwrist1_nodepath.setMat(ur5wrist1_rotmat)
    ur5lftwrist1_nodepath.setColor(.5,.7,.3,1)
    ur5lftwrist1_nodepath.reparentTo(ur5mnp)
    #
    ur5wrist2_model.instanceTo(ur5lftwrist2_nodepath)
    ur5wrist2_rotmat = pg.cvtMat4(ur5dual.lftarm[5]['rotmat'], ur5dual.lftarm[5]['linkpos'])
    ur5lftwrist2_nodepath.setMat(ur5wrist2_rotmat)
    ur5lftwrist2_nodepath.setColor(.5,.5,.5,1)
    ur5lftwrist2_nodepath.reparentTo(ur5mnp)
    #
    ur5wrist3_model.instanceTo(ur5lftwrist3_nodepath)
    ur5wrist3_rotmat = pg.cvtMat4(ur5dual.lftarm[6]['rotmat'], ur5dual.lftarm[6]['linkpos'])
    ur5lftwrist3_nodepath.setMat(ur5wrist3_rotmat)
    ur5lftwrist3_nodepath.setColor(.5,.5,.5,1)
    ur5lftwrist3_nodepath.reparentTo(ur5mnp)

    # rgthnd
    ur5robotrgthnd = handpkg.newHand('rgt')
    ur5robotrgtarmljend_rotmat = pg.cvtMat4(ur5dual.rgtarm[6]['rotmat'], ur5dual.rgtarm[6]['linkpos'])
    pg.plotAxisSelf(ur5mnp, ur5dual.rgtarm[6]['linkend'], ur5robotrgtarmljend_rotmat)
    ur5robotrgthnd.setMat(ur5robotrgtarmljend_rotmat)
    ur5robotrgthnd.reparentTo(ur5mnp)
    if jawwidthrgt is not None:
        ur5robotrgthnd.setJawwidth(jawwidthrgt)

    # lfthnd
    ur5robotlfthnd = handpkg.newHand('lft')
    ur5robotlftarmljend_rotmat = pg.cvtMat4(ur5dual.lftarm[6]['rotmat'], ur5dual.lftarm[6]['linkpos'])
    pg.plotAxisSelf(ur5mnp, ur5dual.lftarm[6]['linkend'], ur5robotlftarmljend_rotmat)
    ur5robotlfthnd.setMat(ur5robotlftarmljend_rotmat)
    ur5robotlfthnd.reparentTo(ur5mnp)
    if jawwidthlft is not None:
        ur5robotlfthnd.setJawwidth(jawwidthlft)

    return ur5mnp


def genmnplist(ur5dual, handpkg, jawwidthrgt = None, jawwidthlft = None):
    """
    generate a list of panda3d nodepath for the ur5dual
    mnp indicates this function generates a mesh nodepath

    the return value is in the following format:
    [[rightarm mnp list], [leftarm mnp list], [body]]
    # Right goes first!
    # The order of an arm mnp list is from base to end-effector

    :param ur5dual:
    :return: a list of mesh nodepaths

    author: weiwei
    date: 20170608
    """

    ur5mnp = NodePath("ur5dualmnp")

    this_dir, this_filename = os.path.split(__file__)

    # chest0-2, head1 (neck is not plotted)
    ur5base_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur5egg", "base.egg"))
    ur5upperarm_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur5egg", "upperarm.egg"))
    ur5shoulder_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur5egg", "shoulder.egg"))
    ur5forearm_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur5egg", "forearm.egg"))
    ur5wrist1_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur5egg", "wrist1.egg"))
    ur5wrist2_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur5egg", "wrist2.egg"))
    ur5wrist3_filepath = Filename.fromOsSpecific(os.path.join(this_dir, "ur5egg", "wrist3.egg"))

    ur5base_model  = loader.loadModel(ur5base_filepath)
    ur5upperarm_model  = loader.loadModel(ur5upperarm_filepath)
    ur5shoulder_model = loader.loadModel(ur5shoulder_filepath)
    ur5forearm_model = loader.loadModel(ur5forearm_filepath)
    ur5wrist1_model = loader.loadModel(ur5wrist1_filepath)
    ur5wrist2_model = loader.loadModel(ur5wrist2_filepath)
    ur5wrist3_model = loader.loadModel(ur5wrist3_filepath)

    # rgt
    ur5rgtbase_nodepath = NodePath("ur5rgtbase")
    ur5rgtshoulder_nodepath = NodePath("ur5rgtshoulder")
    ur5rgtupperarm_nodepath = NodePath("ur5rgtupperarm")
    ur5rgtforearm_nodepath = NodePath("ur5rgtforearm")
    ur5rgtwrist1_nodepath = NodePath("ur5rgtwrist1")
    ur5rgtwrist2_nodepath = NodePath("ur5rgtwrist2")
    ur5rgtwrist3_nodepath = NodePath("ur5rgtwrist3")
    #
    # base egg coordinates is retracted 89.2 along z to fit parameters
    ur5base_model.instanceTo(ur5rgtbase_nodepath)
    ur5base_rotmat = pg.cvtMat4(ur5dual.rgtarm[1]['inherentR'], ur5dual.rgtarm[1]['linkpos'])
    ur5rgtbase_nodepath.setMat(ur5base_rotmat)
    ur5rgtbase_nodepath.setColor(.5,.5,.5,1)
    ur5rgtbase_nodepath.reparentTo(ur5mnp)
    #
    ur5shoulder_model.instanceTo(ur5rgtshoulder_nodepath)
    ur5shoulder_rotmat = pg.cvtMat4(ur5dual.rgtarm[1]['rotmat'], ur5dual.rgtarm[1]['linkpos'])
    ur5rgtshoulder_nodepath.setMat(ur5shoulder_rotmat)
    ur5rgtshoulder_nodepath.setColor(.5,.7,.3,1)
    ur5rgtshoulder_nodepath.reparentTo(ur5mnp)
    #
    ur5upperarm_model.instanceTo(ur5rgtupperarm_nodepath)
    ur5upperarm_rotmat = pg.cvtMat4(ur5dual.rgtarm[2]['rotmat'], ur5dual.rgtarm[2]['linkpos'])
    ur5rgtupperarm_nodepath.setMat(ur5upperarm_rotmat)
    ur5rgtupperarm_nodepath.setColor(.5,.5,.5,1)
    ur5rgtupperarm_nodepath.reparentTo(ur5mnp)
    #
    ur5forearm_model.instanceTo(ur5rgtforearm_nodepath)
    ur5forearm_rotmat = pg.cvtMat4(ur5dual.rgtarm[3]['rotmat'], ur5dual.rgtarm[3]['linkpos'])
    ur5rgtforearm_nodepath.setMat(ur5forearm_rotmat)
    ur5rgtforearm_nodepath.setColor(.5,.5,.5,1)
    ur5rgtforearm_nodepath.reparentTo(ur5mnp)
    #
    ur5wrist1_model.instanceTo(ur5rgtwrist1_nodepath)
    ur5wrist1_rotmat = pg.cvtMat4(ur5dual.rgtarm[4]['rotmat'], ur5dual.rgtarm[4]['linkpos'])
    ur5rgtwrist1_nodepath.setMat(ur5wrist1_rotmat)
    ur5rgtwrist1_nodepath.setColor(.5,.7,.3,1)
    ur5rgtwrist1_nodepath.reparentTo(ur5mnp)
    #
    ur5wrist2_model.instanceTo(ur5rgtwrist2_nodepath)
    ur5wrist2_rotmat = pg.cvtMat4(ur5dual.rgtarm[5]['rotmat'], ur5dual.rgtarm[5]['linkpos'])
    ur5rgtwrist2_nodepath.setMat(ur5wrist2_rotmat)
    ur5rgtwrist2_nodepath.setColor(.5,.5,.5,1)
    ur5rgtwrist2_nodepath.reparentTo(ur5mnp)
    #
    # wrist3 egg coordinates is rotated 90 around z retracted 72.33752-81.82489 to fit parameters
    ur5wrist3_model.instanceTo(ur5rgtwrist3_nodepath)
    ur5wrist3_rotmat = pg.cvtMat4(ur5dual.rgtarm[6]['rotmat'], ur5dual.rgtarm[6]['linkpos'])
    ur5rgtwrist3_nodepath.setMat(ur5wrist3_rotmat)
    ur5rgtwrist3_nodepath.setColor(.5,.5,.5,1)
    ur5rgtwrist3_nodepath.reparentTo(ur5mnp)

    # lft
    ur5lftbase_nodepath = NodePath("ur5lftbase")
    ur5lftupperarm_nodepath = NodePath("ur5lftupperarm")
    ur5lftshoulder_nodepath = NodePath("ur5lftshoulder")
    ur5lftforearm_nodepath = NodePath("ur5lftforearm")
    ur5lftwrist1_nodepath = NodePath("ur5lftwrist1")
    ur5lftwrist2_nodepath = NodePath("ur5lftwrist2")
    ur5lftwrist3_nodepath = NodePath("ur5lftwrist3")
    #
    # base egg coordinates is retracted 89.2 along z to fit parameters
    ur5base_model.instanceTo(ur5lftbase_nodepath)
    ur5base_rotmat = pg.cvtMat4(ur5dual.lftarm[1]['inherentR'], ur5dual.lftarm[1]['linkpos'])
    ur5lftbase_nodepath.setMat(ur5base_rotmat)
    ur5lftbase_nodepath.setColor(.5,.5,.5,1)
    ur5lftbase_nodepath.reparentTo(ur5mnp)
    #
    ur5shoulder_model.instanceTo(ur5lftshoulder_nodepath)
    ur5shoulder_rotmat = pg.cvtMat4(ur5dual.lftarm[1]['rotmat'], ur5dual.lftarm[1]['linkpos'])
    ur5lftshoulder_nodepath.setMat(ur5shoulder_rotmat)
    ur5lftshoulder_nodepath.setColor(.5,.7,.3,1)
    ur5lftshoulder_nodepath.reparentTo(ur5mnp)
    #
    ur5upperarm_model.instanceTo(ur5lftupperarm_nodepath)
    ur5upperarm_rotmat = pg.cvtMat4(ur5dual.lftarm[2]['rotmat'], ur5dual.lftarm[2]['linkpos'])
    ur5lftupperarm_nodepath.setMat(ur5upperarm_rotmat)
    ur5lftupperarm_nodepath.setColor(.5,.5,.5,1)
    ur5lftupperarm_nodepath.reparentTo(ur5mnp)
    #
    ur5forearm_model.instanceTo(ur5lftforearm_nodepath)
    ur5forearm_rotmat = pg.cvtMat4(ur5dual.lftarm[3]['rotmat'], ur5dual.lftarm[3]['linkpos'])
    ur5lftforearm_nodepath.setMat(ur5forearm_rotmat)
    ur5lftforearm_nodepath.setColor(.5,.5,.5,1)
    ur5lftforearm_nodepath.reparentTo(ur5mnp)
    #
    ur5wrist1_model.instanceTo(ur5lftwrist1_nodepath)
    ur5wrist1_rotmat = pg.cvtMat4(ur5dual.lftarm[4]['rotmat'], ur5dual.lftarm[4]['linkpos'])
    ur5lftwrist1_nodepath.setMat(ur5wrist1_rotmat)
    ur5lftwrist1_nodepath.setColor(.5,.7,.3,1)
    ur5lftwrist1_nodepath.reparentTo(ur5mnp)
    #
    ur5wrist2_model.instanceTo(ur5lftwrist2_nodepath)
    ur5wrist2_rotmat = pg.cvtMat4(ur5dual.lftarm[5]['rotmat'], ur5dual.lftarm[5]['linkpos'])
    ur5lftwrist2_nodepath.setMat(ur5wrist2_rotmat)
    ur5lftwrist2_nodepath.setColor(.5,.5,.5,1)
    ur5lftwrist2_nodepath.reparentTo(ur5mnp)
    #
    ur5wrist3_model.instanceTo(ur5lftwrist3_nodepath)
    ur5wrist3_rotmat = pg.cvtMat4(ur5dual.lftarm[6]['rotmat'], ur5dual.lftarm[6]['linkpos'])
    ur5lftwrist3_nodepath.setMat(ur5wrist3_rotmat)
    ur5lftwrist3_nodepath.setColor(.5,.5,.5,1)
    ur5lftwrist3_nodepath.reparentTo(ur5mnp)

    # rgthnd
    ur5robotrgthnd = handpkg.newHand('rgt')
    ur5robotrgtarmljend_rotmat = pg.cvtMat4(ur5dual.rgtarm[6]['rotmat'], ur5dual.rgtarm[6]['linkpos'])
    pg.plotAxisSelf(ur5mnp, ur5dual.rgtarm[6]['linkend'], ur5robotrgtarmljend_rotmat)
    ur5robotrgthnd.setMat(ur5robotrgtarmljend_rotmat)
    ur5robotrgthnd.reparentTo(ur5mnp)
    if jawwidthrgt is not None:
        ur5robotrgthnd.setJawwidth(jawwidthrgt)

    # lfthnd
    ur5robotlfthnd = handpkg.newHand('lft')
    ur5robotlftarmljend_rotmat = pg.cvtMat4(ur5dual.lftarm[6]['rotmat'], ur5dual.lftarm[6]['linkpos'])
    pg.plotAxisSelf(ur5mnp, ur5dual.lftarm[6]['linkend'], ur5robotlftarmljend_rotmat)
    ur5robotlfthnd.setMat(ur5robotlftarmljend_rotmat)
    ur5robotlfthnd.reparentTo(ur5mnp)
    if jawwidthlft is not None:
        ur5robotlfthnd.setJawwidth(jawwidthlft)

    return [[ur5rgtbase_nodepath, ur5rgtshoulder_nodepath, ur5rgtupperarm_nodepath,
             ur5rgtforearm_nodepath, ur5rgtwrist1_nodepath, ur5rgtwrist2_nodepath,
             ur5rgtwrist3_nodepath, ur5robotrgthnd.handnp],
            [ur5lftbase_nodepath, ur5lftshoulder_nodepath, ur5lftupperarm_nodepath,
             ur5lftforearm_nodepath, ur5lftwrist1_nodepath, ur5lftwrist2_nodepath,
             ur5lftwrist3_nodepath, ur5robotlfthnd.handnp],
            []]

def genmnp_nm(ur5dual, handpkg, jawwidthrgt = None, jawwidthlft = None):
    """
    generate a panda3d nodepath for the ur5dual
    mnp indicates this function generates a mesh nodepath

    :param ur5dual:
    :return: a nodepath which is ready to be plotted using plotmesh

    author: weiwei
    date: 20161202
    """

    return genmnp(ur5dual, handpkg, jawwidthrgt, jawwidthlft)