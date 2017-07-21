# plot the shortest path using hrp5robot

from robotsim.nextage import nxt
from robotsim.nextage import nxtplot

from manipulation.grip.robotiq85 import rtq85nm
from manipulation.regrasp import regriptppass
from database import dbaccess as db
import pandaplotutils.pandactrl as pandactrl
from panda3d.core import *
from pandaplotutils import pandageom as pg
import matplotlib.pyplot as plt
import os
import numpy as np
import utils.robotmath as rm

def genMotionSequence(regrip, id):
    """
    generate motion sequence using the shortest path
    right arm

    :param: regrip an object of the regriptppfp.RegripTppfp class
    :param id: which path to plot

    :return: [[waist, lftbody, rgtbody],...]

    author: weiwei
    date: 20170309
    """

    if len(regrip.directshortestpaths) == 0:
        print "no path found"
        return

    pathnidlist = regrip.directshortestpaths[id]
    # divide path
    pathnidlist0 = []
    pathnidlist1 = []
    for nid in pathnidlist:
        pathnidlist0.append(nid)
        if nid.startswith('assrgt'):
            break
    for nid in pathnidlist[::-1]:
        pathnidlist1.append(nid)
        if nid.startswith('asslft'):
            break
    print pathnidlist0
    print pathnidlist1
    # rgthnd
    numikrlist0, objmat4list0, jawwidthlist0 = genMotionSequenceSgl(pathnidlist0, 'rgt')
    # lfthnd
    numikrlist1, objmat4list1, jawwidthlist1 = genMotionSequenceSgl(pathnidlist1, 'lft')

    print numikrlist0
    print numikrlist1
    # rgt first lft second, update lft with rgt
    numikr0last = numikrlist0[-1]
    jawwidth0last = jawwidthlist0[-1]
    for i in range(len(numikrlist1)):
        numikrlist1[i][1][0] = numikr0last[1][0]
        numikrlist1[i][1][1] = numikr0last[1][1]
        numikrlist1[i][1][2] = numikr0last[1][2]
        numikrlist1[i][1][3] = numikr0last[1][3]
        numikrlist1[i][1][4] = numikr0last[1][4]
        numikrlist1[i][1][5] = numikr0last[1][5]
        jawwidthlist1[i][0] = jawwidth0last[0]
    for i in range(len(numikrlist1)):
        objmat4list0.append(objmat4list0[-1])
    for i in range(len(numikrlist0)):
        objmat4list1.insert(0, objmat4list1[0])

    numikrlist = numikrlist0 + numikrlist1
    jawwidthlist = jawwidthlist0 + jawwidthlist1

    # update objects in rigth arm when moving left arm
    for i in range(len(numikrlist1)):
        # WARN: only upward robot!
        baseangle = numikrlist1[i][0]
        updateobjrotmat = objmat4list0[len(numikrlist0)+i]*Mat4.rotateMatNormaxis(baseangle, Vec3(0,0,1))
        objmat4list0[len(numikrlist0) + i] = updateobjrotmat

    # final dual-arm assembly
    nid0 = pathnidlist0[-1]
    grppos = regrip.regg.node[nid0]['fgrcenter']
    grprot = regrip.regg.node[nid0]['hndrotmat3np']
    jawwidth0 = regrip.regg.node[nid0]['jawwidth']
    armjntsgrp0 = regrip.robot.numik(grppos, grprot, armid = 'rgt')
    objmat40 = regrip.regg.node[nid0]['assposerotmat4']
    nid1 = pathnidlist1[-1]
    grppos = regrip.regg.node[nid1]['fgrcenter']
    grprot = regrip.regg.node[nid1]['hndrotmat3np']
    jawwidth1 = regrip.regg.node[nid1]['jawwidth']
    armjntsgrp1 = regrip.robot.numik(grppos, grprot, armid = 'lft')
    objmat41 = regrip.regg.node[nid1]['assposerotmat4']
    numikrlist.append([0, armjntsgrp0, armjntsgrp1])
    objmat4list0.append(objmat40)
    objmat4list1.append(objmat41)
    jawwidthlist.append([jawwidth0, jawwidth1])

    return [numikrlist, objmat4list0, objmat4list1, jawwidthlist]


def genMotionSequenceSgl(pathnidlist, armname):
    """
    generate assembly motion sequence for a single arm

    :param pathnidlist: half of the path list starting with "endxxx" and end with "assxxx"
    :param armname:
    :return: [numikrlist, objmat4list, jawwidthlist]
            where numikrlist = [waist, rgtarm, lftarm]
    """

    numikrlist = []
    objmat4list = []
    jawwidth = []

    for i in range(len(pathnidlist) - 1):
        if i == 0 and len(pathnidlist) == 2:
            # two node path
            # one must be end, the other must be ass
            ## starting node
            nid0 = pathnidlist[i]
            grpposhandx = regrip.regg.node[nid0]['fgrcenterhandx']
            grppos = regrip.regg.node[nid0]['fgrcenter']
            grpposworlda = regrip.regg.node[nid0]['fgrcenterworlda']
            grpposworldaworldz = regrip.regg.node[nid0]['fgrcenterworldaworldz']
            grprot = regrip.regg.node[nid0]['hndrotmat3np']
            grpjawwidth = regrip.regg.node[nid0]['jawwidth']
            armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot, armid = armname)
            armjntsgrp = regrip.robot.numikr(grppos, grprot, armid = armname)
            armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot, armid = armname)
            armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot, armid = armname)
            if armname == 'rgt':
                numikrlist.append([armjntsgrphandx[0], armjntsgrphandx[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrpworlda[0], armjntsgrpworlda[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrpworldaworldz[0], armjntsgrpworldaworldz[1], regrip.robot.initlftjnts])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
            else:
                numikrlist.append([armjntsgrphandx[0], regrip.robot.initrgtjnts, armjntsgrphandx[1]])
                numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                numikrlist.append([armjntsgrpworlda[0], regrip.robot.initrgtjnts, armjntsgrpworlda[1]])
                numikrlist.append([armjntsgrpworldaworldz[0], regrip.robot.initrgtjnts, armjntsgrpworldaworldz[1]])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
            objmat4handx = regrip.regg.node[nid0]['tabletopplacementrotmathandx']
            objmat4 = regrip.regg.node[nid0]['tabletopplacementrotmat']
            objmat4worlda = regrip.regg.node[nid0]['tabletopplacementrotmatworlda']
            objmat4worldaworldz = regrip.regg.node[nid0]['tabletopplacementrotmatworldaworldz']
            objmat4list.append(objmat4handx)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4worlda)
            objmat4list.append(objmat4worldaworldz)
            ## first node
            nid1 = pathnidlist[i + 1]
            grpposworlda = regrip.regg.node[nid1]['fgrcenterretass']
            # grppos = regrip.regg.node[nid1]['fgrcenter']
            grprot = regrip.regg.node[nid1]['hndrotmat3np']
            grpjawwidth = regrip.regg.node[nid1]['jawwidth']
            armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot, armid = armname)
            # armjntsgrp = regrip.robot.numik(grppos, grprot, armid = armname)
            if armname == 'rgt':
                numikrlist.append([0, armjntsgrpworlda, regrip.robot.initlftjnts])
                # numikrlist.append(np.concatenate([np.append(0, armjntsgrp), regrip.robot.initlftjnts]))
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                # jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
            else:
                numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrpworlda])
                # numikrlist.append(np.concatenate([regrip.robot.initrgtjntsr, armjntsgrp]))
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                # jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
            objmat4 = regrip.regg.node[nid1]['assposerotmat4retass']
            objmat4list.append(objmat4)
        elif i == 0:
            # not two nodepath, starting node, transfer
            ## starting node
            nid0 = pathnidlist[i]
            grpposhandx = regrip.regg.node[nid0]['fgrcenterhandx']
            grppos = regrip.regg.node[nid0]['fgrcenter']
            grpposworlda = regrip.regg.node[nid0]['fgrcenterworlda']
            grpposworldaworldz = regrip.regg.node[nid0]['fgrcenterworldaworldz']
            grprot = regrip.regg.node[nid0]['hndrotmat3np']
            grpjawwidth = regrip.regg.node[nid0]['jawwidth']
            armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot, armid=armname)
            armjntsgrp = regrip.robot.numikr(grppos, grprot, armid=armname)
            armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot, armid=armname)
            armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot, armid=armname)
            if armname == 'rgt':
                numikrlist.append([armjntsgrphandx[0], armjntsgrphandx[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrpworlda[0], armjntsgrpworlda[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrpworldaworldz[0], armjntsgrpworldaworldz[1], regrip.robot.initlftjnts])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
            else:
                numikrlist.append([armjntsgrphandx[0], regrip.robot.initrgtjnts, armjntsgrphandx[1]])
                numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                numikrlist.append([armjntsgrpworlda[0], regrip.robot.initrgtjnts, armjntsgrpworlda[1]])
                numikrlist.append([armjntsgrpworldaworldz[0], regrip.robot.initrgtjnts, armjntsgrpworldaworldz[1]])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
            objmat4handx = regrip.regg.node[nid0]['tabletopplacementrotmathandx']
            objmat4 = regrip.regg.node[nid0]['tabletopplacementrotmat']
            objmat4worlda = regrip.regg.node[nid0]['tabletopplacementrotmatworlda']
            objmat4worldaworldz = regrip.regg.node[nid0]['tabletopplacementrotmatworldaworldz']
            objmat4list.append(objmat4handx)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4worlda)
            objmat4list.append(objmat4worldaworldz)
            ## first node
            nid1 = pathnidlist[i + 1]
            grpposworldaworldz = regrip.regg.node[nid1]['fgrcenterworldaworldz']
            grpposworlda = regrip.regg.node[nid1]['fgrcenterworlda']
            grppos = regrip.regg.node[nid1]['fgrcenter']
            grpposhandx = regrip.regg.node[nid1]['fgrcenterhandx']
            grpposhandxworldz = regrip.regg.node[nid1]['fgrcenterhandxworldz']
            grprot = regrip.regg.node[nid1]['hndrotmat3np']
            grpjawwidth = regrip.regg.node[nid1]['jawwidth']
            armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot, armid = armname)
            armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot, armid = armname)
            armjntsgrp = regrip.robot.numik(grppos, grprot, armid = armname)
            armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot, armid = armname)
            armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot, armid = armname)
            if armname == 'rgt':
                numikrlist.append([0, armjntsgrpworldaworldz, regrip.robot.initlftjnts])
                numikrlist.append([0, armjntsgrpworlda, regrip.robot.initlftjnts])
                numikrlist.append([0, armjntsgrp, regrip.robot.initlftjnts])
                numikrlist.append([0, armjntsgrp, regrip.robot.initlftjnts])
                numikrlist.append([0, armjntsgrphandx, regrip.robot.initlftjnts])
                numikrlist.append([0, armjntsgrphandxworldz, regrip.robot.initlftjnts])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
            else:
                numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrpworldaworldz])
                numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrpworlda])
                numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrphandx])
                numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrphandxworldz])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
            objmat4worldaworldz = regrip.regg.node[nid1]['tabletopplacementrotmatworldaworldz']
            objmat4worlda = regrip.regg.node[nid1]['tabletopplacementrotmatworlda']
            objmat4 = regrip.regg.node[nid1]['tabletopplacementrotmat']
            objmat4handx = regrip.regg.node[nid1]['tabletopplacementrotmathandx']
            objmat4handxworldz = regrip.regg.node[nid1]['tabletopplacementrotmathandxworldz']
            objmat4list.append(objmat4worldaworldz)
            objmat4list.append(objmat4worlda)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4handx)
            objmat4list.append(objmat4handxworldz)
        elif i + 1 != len(pathnidlist) - 1:
            # not two node path, middle nodes, if transfer
            if regrip.regg.edge[pathnidlist[i]][pathnidlist[i+1]]['edgetype'].endswith("transit"):
                pass
            else:
                ## middle first
                nid0 = pathnidlist[i]
                grpposhandxworldz = regrip.regg.node[nid0]['fgrcenterhandxworldz']
                grpposhandx = regrip.regg.node[nid0]['fgrcenterhandx']
                grppos = regrip.regg.node[nid0]['fgrcenter']
                grpposworlda = regrip.regg.node[nid0]['fgrcenterworlda']
                grpposworldaworldz = regrip.regg.node[nid0]['fgrcenterworldaworldz']
                grprot = regrip.regg.node[nid0]['hndrotmat3np']
                grpjawwidth = regrip.regg.node[nid0]['jawwidth']
                armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot, armid = armname)
                armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot, armid = armname)
                armjntsgrp = regrip.robot.numik(grppos, grprot, armid = armname)
                armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot, armid = armname)
                armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot, armid = armname)
                if armname == 'rgt':
                    numikrlist.append([0, armjntsgrphandxworldz, regrip.robot.initlftjnts])
                    numikrlist.append([0, armjntsgrphandx, regrip.robot.initlftjnts])
                    numikrlist.append([0, armjntsgrp, regrip.robot.initlftjnts])
                    numikrlist.append([0, armjntsgrp, regrip.robot.initlftjnts])
                    numikrlist.append([0, armjntsgrpworlda, regrip.robot.initlftjnts])
                    numikrlist.append([0, armjntsgrpworldaworldz, regrip.robot.initlftjnts])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                else:
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrphandxworldz])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrphandx])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrpworlda])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrpworldaworldz])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                objmat4handxworldz = regrip.regg.node[nid0]['tabletopplacementrotmathandxworldz']
                objmat4handx = regrip.regg.node[nid0]['tabletopplacementrotmathandx']
                objmat4 = regrip.regg.node[nid0]['tabletopplacementrotmat']
                objmat4worlda = regrip.regg.node[nid0]['tabletopplacementrotmatworlda']
                objmat4worldaworldz = regrip.regg.node[nid0]['tabletopplacementrotmatworldaworldz']
                objmat4list.append(objmat4handxworldz)
                objmat4list.append(objmat4handx)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4worlda)
                objmat4list.append(objmat4worldaworldz)
                ## middle second
                nid1 = pathnidlist[i + 1]
                grpposworldaworldz = regrip.regg.node[nid1]['fgrcenterworldaworldz']
                grpposworlda = regrip.regg.node[nid1]['fgrcenterworlda']
                grppos = regrip.regg.node[nid1]['fgrcenter']
                grpposhandx = regrip.regg.node[nid1]['fgrcenterhandx']
                grpposhandxworldz = regrip.regg.node[nid1]['fgrcenterhandxworldz']
                grprot = regrip.regg.node[nid1]['hndrotmat3np']
                grpjawwidth = regrip.regg.node[nid1]['jawwidth']
                armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot, armid = armname)
                armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot, armid = armname)
                armjntsgrp = regrip.robot.numik(grppos, grprot, armid = armname)
                armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot, armid = armname)
                armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot, armid = armname)
                if armname == 'rgt':
                    numikrlist.append([0, armjntsgrpworldaworldz, regrip.robot.initlftjnts])
                    numikrlist.append([0, armjntsgrpworlda, regrip.robot.initlftjnts])
                    numikrlist.append([0, armjntsgrp, regrip.robot.initlftjnts])
                    numikrlist.append([0, armjntsgrp, regrip.robot.initlftjnts])
                    numikrlist.append([0, armjntsgrphandx, regrip.robot.initlftjnts])
                    numikrlist.append([0, armjntsgrphandxworldz, regrip.robot.initlftjnts])
                    jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                else:
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrpworldaworldz])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrpworlda])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrphandx])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrphandxworldz])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                objmat4worldaworldz = regrip.regg.node[nid1]['tabletopplacementrotmatworldaworldz']
                objmat4worlda = regrip.regg.node[nid1]['tabletopplacementrotmatworlda']
                objmat4 = regrip.regg.node[nid1]['tabletopplacementrotmat']
                objmat4handx = regrip.regg.node[nid1]['tabletopplacementrotmathandx']
                objmat4handxworldz = regrip.regg.node[nid1]['tabletopplacementrotmathandxworldz']
                objmat4list.append(objmat4worldaworldz)
                objmat4list.append(objmat4worlda)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4handx)
                objmat4list.append(objmat4handxworldz)
        else:
            # not two node path, end nodes, transfer
            ## second to last node
            nid0 = pathnidlist[i]
            grpposhandxworldz = regrip.regg.node[nid0]['fgrcenterhandxworldz']
            grpposhandx = regrip.regg.node[nid0]['fgrcenterhandx']
            grppos = regrip.regg.node[nid0]['fgrcenter']
            grpposworlda = regrip.regg.node[nid0]['fgrcenterworlda']
            grpposworldaworldz = regrip.regg.node[nid0]['fgrcenterworldaworldz']
            grprot = regrip.regg.node[nid0]['hndrotmat3np']
            grpjawwidth = regrip.regg.node[nid0]['jawwidth']
            armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot, armid = armname)
            armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot, armid = armname)
            armjntsgrp = regrip.robot.numik(grppos, grprot, armid = armname)
            armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot, armid = armname)
            armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot, armid = armname)
            if armname == 'rgt':
                numikrlist.append([0, armjntsgrphandxworldz, regrip.robot.initlftjnts])
                numikrlist.append([0, armjntsgrphandx, regrip.robot.initlftjnts])
                numikrlist.append([0, armjntsgrp, regrip.robot.initlftjnts])
                numikrlist.append([0, armjntsgrp, regrip.robot.initlftjnts])
                numikrlist.append([0, armjntsgrpworlda, regrip.robot.initlftjnts])
                numikrlist.append([0, armjntsgrpworldaworldz, regrip.robot.initlftjnts])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
            else:
                numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrphandxworldz])
                numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrphandx])
                numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp])
                numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrpworlda])
                numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrpworldaworldz])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
            objmat4handxworldz = regrip.regg.node[nid0]['tabletopplacementrotmathandxworldz']
            objmat4handx = regrip.regg.node[nid0]['tabletopplacementrotmathandx']
            objmat4 = regrip.regg.node[nid0]['tabletopplacementrotmat']
            objmat4worlda = regrip.regg.node[nid0]['tabletopplacementrotmatworlda']
            objmat4worldaworldz = regrip.regg.node[nid0]['tabletopplacementrotmatworldaworldz']
            objmat4list.append(objmat4handxworldz)
            objmat4list.append(objmat4handx)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4worlda)
            objmat4list.append(objmat4worldaworldz)
            ## last node
            nid1 = pathnidlist[i + 1]
            grpposworlda = regrip.regg.node[nid1]['fgrcenterretass']
            # grppos = regrip.regg.node[nid1]['fgrcenter']
            grprot = regrip.regg.node[nid1]['hndrotmat3np']
            grpjawwidth = regrip.regg.node[nid1]['jawwidth']
            armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot, armid = armname)
            # armjntsgrp = regrip.robot.numikr(grppos, grprot, armid = armname)
            if armname == 'rgt':
                numikrlist.append([0, armjntsgrpworlda, regrip.robot.initlftjnts])
                # numikrlist.append(np.concatenate([np.append(0, armjntsgrp), regrip.robot.initlftjnts]))
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                # jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
            else:
                numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrpworlda])
                # numikrlist.append(np.concatenate([regrip.robot.initrgtjntsr, armjntsgrp]))
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                # jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
            objmat4 = regrip.regg.node[nid1]['assposerotmat4retass']
            objmat4list.append(objmat4)
    return [numikrlist, objmat4list, jawwidth]

if __name__=='__main__':
    gdb = db.GraspDB()
    handpkg = rtq85nm
    nxtrobot = nxt.NxtRobot()

    # base = pandactrl.World(camp=[0,0,5000], lookatp=[0,0,0])
    base = pandactrl.World(camp=[3000,0,1000], lookatp=[0,0,400])

    this_dir, this_filename = os.path.split(__file__)
    obj0path = os.path.join(os.path.split(os.path.split(this_dir)[0])[0]+os.sep, "grip", "objects", "planefrontstay.stl")
    obj0Mat4 = Mat4.identMat()
    obj1path = os.path.join(os.path.split(os.path.split(this_dir)[0])[0]+os.sep, "grip", "objects", "planewheel.stl")
    obj1Mat4 = Mat4(obj0Mat4)
    obj1Mat4.setCell(3,1,32)
    obj1Mat4.setCell(3,2,10)

    # sprotmat4 = Mat4(0.0,0.0,-1.0,0.0,\
    #                  0.0,-1.0,0.0,0.0,\
    #                  -1.0,0.0,0.0,0.0,\
    #                  400,-200,-37.0,1.0)
    #
    # # sprotmat4 = Mat4(1.0,0.0,0.0,0.0,\
    # #                  0.0,1.0,0.0,0.0,\
    # #                  0.0,0.0,1.0,0.0,\
    # #                  350,-200,0.0,1.0)
    # whrotmat4 = Mat4(1.0,0.0,0.0,0.0,\
    #                  0.0,0.0,1.0,0.0,\
    #                  0.0,-1.0,0.0,0.0,\
    #                  500,350,-50.0,1.0)
    # assDirect1to0 = Vec3(0, -70, 0)

    sprotmat4 = Mat4(0.0,0.0,1.0,0.0,\
                     0.0,1.0,0.0,0.0,\
                     -1.0,0.0,0.0,0.0,\
                     400,-300,-37.0,1.0)

    # sprotmat4 = Mat4(1.0,0.0,0.0,0.0,\
    #                  0.0,1.0,0.0,0.0,\
    #                  0.0,0.0,1.0,0.0,\
    #                  350,-200,0.0,1.0)
    # whrotmat4 = Mat4(1.0,0.0,0.0,0.0,\
    #                  0.0,0.0,1.0,0.0,\
    #                  0.0,-1.0,0.0,0.0,\
    #                  400,200,-50.0,1.0)
    # id = 27
    whrotmat4 = Mat4(1.0,0.0,0.0,0.0,\
                     0.0,0.0,1.0,0.0,\
                     0.0,-1.0,0.0,0.0,\
                     400,350,-50.0,1.0)
    id = 15
    assDirect1to0 = Vec3(0, -70, 0)

    regrip = regriptppass.RegripTppAss(base, obj0path, obj0Mat4, obj1path, obj1Mat4, assDirect1to0, gdb, nxtrobot, handpkg)

    import time
    tic = time.clock()
    regrip.findshortestpath(obj0SRotmat4 = sprotmat4, obj1SRotmat4 = whrotmat4)
    toc = time.clock()
    print toc-tic
    # assert False
    #
    pltfig = plt.figure()
    ax = pltfig.add_subplot(111)
    regrip.plotGraph(ax)
    regrip.plotshortestpath(ax, id = id)
    plt.axis("equal")
    plt.show()
    # set execute
    import robotcon.nextage as nxtcon
    import robotcon.rtq85 as rtq85con
    nxts = nxtcon.NxtSocket()
    rtq85rs = rtq85con.Rtq85Socket(handname = 'rgt')
    rtq85ls = rtq85con.Rtq85Socket(handname = 'lft')
    rtq85rs.initialize()
    rtq85ls.initialize()
    rtq85rs.openhandto(regrip.robothand.jawwidthopen)
    rtq85ls.openhandto(regrip.robothand.jawwidthopen)
    nxts.initialize()
    # values
    [numikrms, objms0, objms1, jawwidth] = genMotionSequence(regrip, id = id)
    nxtmnp = [None]
    obj0mnp = [None]
    obj1mnp = [None]
    counter = [0]
    def updateshow(objms0, objms1, numikrms, jawwidth, nxtmnp, obj0mnp, obj1mnp, counter, nxtrobot, obj0path, obj1path, task):
        if counter[0] < len(numikrms):
            if nxtmnp[0] is not None:
                nxtmnp[0].detachNode()
            if obj0mnp[0] is not None:
                obj0mnp[0].detachNode()
            if obj1mnp[0] is not None:
                obj1mnp[0].detachNode()
            alljnts = [numikrms[counter[0]][0], 0, 0]
            alljnts.extend([i for i in numikrms[counter[0]][1]])
            alljnts.extend([i for i in numikrms[counter[0]][2]])
            nxtrobot.movealljnts(alljnts)
            nxtmnp[0] = nxtplot.genmnp(nxtrobot, handpkg, jawwidthrgt=jawwidth[counter[0]][0], jawwidthlft=jawwidth[counter[0]][1])
            nxtrobot.goinitpose()
            nxtmnp[0].reparentTo(base.render)
            obj0mnp[0] = pg.genObjmnp(obj0path, color = Vec4(.7,.3,0,1))
            obj0mnp[0].setMat(objms0[counter[0]])
            # pg.plotAxisSelf(base.render,objms0[counter[0]].getRow3(3), objms[counter[0]])
            obj0mnp[0].reparentTo(base.render)
            obj1mnp[0] = pg.genObjmnp(obj1path, color = Vec4(0,.3,.7,1))
            obj1mnp[0].setMat(objms1[counter[0]])
            # pg.plotAxisSelf(base.render,objms1[counter[0]].getRow3(3), objms[counter[0]])
            obj1mnp[0].reparentTo(base.render)
            if counter[0] >= 1:
                print base.inputmgr.keyMap['space']
                if base.inputmgr.keyMap['space'] is False:
                    pass
                else:
                    c2e = counter[0]-1
                    alljnts = [numikrms[c2e][0], 0, 0]
                    alljnts.extend([i for i in numikrms[c2e][1]])
                    alljnts.extend([i for i in numikrms[c2e][2]])
                    nxts.movejnts15(alljnts)
                    if jawwidth[c2e][0] < 85:
                        rtq85rs.openhandto(0)
                    else:
                        rtq85rs.openhandto(85)
                    if jawwidth[c2e][1] < 85:
                        rtq85ls.openhandto(0)
                    else:
                        rtq85ls.openhandto(85)
                    counter[0] += 1
                    base.inputmgr.keyMap['space'] = False
            else:
                counter[0] += 1
        else:
            if base.inputmgr.keyMap['space'] is True:
                c2e = counter[0]-1
                alljnts = [numikrms[c2e][0], 0, 0]
                alljnts.extend([i for i in numikrms[c2e][1]])
                alljnts.extend([i for i in numikrms[c2e][2]])
                # nxts.movejnts15(alljnts)
                if jawwidth[c2e][0] < 85:
                    rtq85rs.openhandto(0)
                else:
                    rtq85rs.openhandto(85)
                if jawwidth[c2e][1] < 85:
                    rtq85ls.openhandto(0)
                else:
                    rtq85ls.openhandto(85)
                # if nxtmnp[0] is not None:
                #     nxtmnp[0].detachNode()
                # nxtrobot.goinitpose()
                # nxtmnp[0] = nxtplot.genNxtmnp(nxtrobot)
                # nxtmnp[0].reparentTo(base.render)
                # counter[0] = 0
                return task.done

        return task.again


    taskMgr.add(updateshow, "updateshow",
                extraArgs = [objms0, objms1, numikrms, jawwidth, nxtmnp, obj0mnp, obj1mnp, counter, nxtrobot, obj0path, obj1path],
                appendTask = True)

    # # plot one frame

    base.run()