# plot the shortest path using hrp5robot

from robotsim.nextage import nxt
from robotsim.nextage import nxtplot
from robotsim.hrp5n import hrp5n
from robotsim.hrp5n import hrp5nplot

from manipulation.grip.robotiq85 import rtq85nm
from manipulation.grip.hrp5three import hrp5threenm
from manipulation.regrasp import regriptppfp
from database import dbaccess as db
import pandaplotutils.pandactrl as pandactrl
from panda3d.core import *
from pandaplotutils import pandageom as pg
import matplotlib.pyplot as plt
import os
import numpy as np

def getMotionSequence(regrip, id, choice):
    """
    generate motion sequence using the shortest path
    right arm

    :param: regrip an object of the regriptppfp.RegripTppfp class
    :param id: which path to plot
    :param choice: startrgtgoalrgt/startrgtgoallft/startlftgoalrgt/startlftgoallft

    :return: [[waist, lftbody, rgtbody],...]

    author: weiwei
    date: 20170302
    """

    directshortestpaths = []
    if choice is 'startrgtgoalrgt':
        directshortestpaths = regrip.directshortestpaths_startrgtgoalrgt
    elif choice is 'startrgtgoallft':
        directshortestpaths = regrip.directshortestpaths_startrgtgoallft
    elif choice is 'startlftgoalrgt':
        directshortestpaths = regrip.directshortestpaths_startlftgoalrgt
    elif choice is 'startlftgoallft':
        directshortestpaths = regrip.directshortestpaths_startlftgoallft

    if len(directshortestpaths) == 0:
        print "no path found"
        return

    pathnidlist = directshortestpaths[id]
    numikrlist = []
    objmat4list = []
    jawwidth = []
    for i in range(len(pathnidlist) - 1):
        if i == 0 and len(pathnidlist) == 2:
            # two node path
            # they must be both rgt or both lft
            # they cannot be handover
            ## starting node
            nid = pathnidlist[i]
            grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
            grppos = regrip.regg.node[nid]['fgrcenter']
            grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
            grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
            grprot = regrip.regg.node[nid]['hndrotmat3np']
            grpjawwidth = regrip.regg.node[nid]['jawwidth']
            # choice
            if nid.startswith('startrgt'):
                armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot, armid = 'rgt')
                armjntsgrp = regrip.robot.numikr(grppos, grprot, armid = 'rgt')
                armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot, armid = 'rgt')
                armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot, armid = 'rgt')
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
            if nid.startswith('startlft'):
                armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot, armid = 'lft')
                armjntsgrp = regrip.robot.numikr(grppos, grprot, armid = 'lft')
                armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot, armid = 'lft')
                armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot, armid = 'lft')
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
            objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
            objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
            objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
            objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
            objmat4list.append(objmat4handx)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4worlda)
            objmat4list.append(objmat4worldaworldz)
            ## first node
            nid = pathnidlist[i + 1]
            grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
            grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
            grppos = regrip.regg.node[nid]['fgrcenter']
            grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
            grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
            grprot = regrip.regg.node[nid]['hndrotmat3np']
            grpjawwidth = regrip.regg.node[nid]['jawwidth']
            # initialize
            # choice
            if nid.startswith('goalrgt'):
                armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot, armid = 'rgt')
                armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot, armid = 'rgt')
                armjntsgrp = regrip.robot.numikr(grppos, grprot, armid = 'rgt')
                armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot, armid = 'rgt')
                armjntsgrphandxworldz = regrip.robot.numikr(grpposhandxworldz, grprot, armid = 'rgt')
                numikrlist.append([armjntsgrpworldaworldz[0], armjntsgrpworldaworldz[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrpworlda[0], armjntsgrpworlda[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrphandx[0], armjntsgrphandx[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrphandxworldz[0], armjntsgrphandxworldz[1], regrip.robot.initlftjnts])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
            if nid.startswith('goallft'):
                armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot, armid = 'lft')
                armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot, armid = 'lft')
                armjntsgrp = regrip.robot.numikr(grppos, grprot, armid = 'lft')
                armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot, armid = 'lft')
                armjntsgrphandxworldz = regrip.robot.numikr(grpposhandxworldz, grprot, armid = 'lft')
                numikrlist.append([armjntsgrpworldaworldz[0], regrip.robot.initrgtjnts, armjntsgrpworldaworldz[1]])
                numikrlist.append([armjntsgrpworlda[0], regrip.robot.initrgtjnts, armjntsgrpworlda[1]])
                numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                numikrlist.append([armjntsgrphandx[0], regrip.robot.initrgtjnts, armjntsgrphandx[1]])
                numikrlist.append([armjntsgrphandxworldz[0], regrip.robot.initrgtjnts, armjntsgrphandxworldz[1]])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
            objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
            objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
            objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
            objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
            objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
            objmat4list.append(objmat4worldaworldz)
            objmat4list.append(objmat4worlda)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4handx)
            objmat4list.append(objmat4handxworldz)
        elif i == 0:
            # not two nodepath, starting node, transfer
            ## starting node
            nid = pathnidlist[i]
            grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
            grppos = regrip.regg.node[nid]['fgrcenter']
            grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
            grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
            grprot = regrip.regg.node[nid]['hndrotmat3np']
            grpjawwidth = regrip.regg.node[nid]['jawwidth']
            # initialize
            armjntsgrphandx = []
            armjntsgrp = []
            armjntsgrpworlda = []
            armjntsgrpworldaworldz = []
            # choice
            if nid.startswith('startrgt'):
                armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot, armid = 'rgt')
                armjntsgrp = regrip.robot.numikr(grppos, grprot, armid = 'rgt')
                armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot, armid = 'rgt')
                armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot, armid = 'rgt')
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
            if nid.startswith('startlft'):
                armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot, armid = 'lft')
                armjntsgrp = regrip.robot.numikr(grppos, grprot, armid = 'lft')
                armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot, armid = 'lft')
                armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot, armid = 'lft')
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
            objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
            objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
            objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
            objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
            objmat4list.append(objmat4handx)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4worlda)
            objmat4list.append(objmat4worldaworldz)
            ## first node
            nid = pathnidlist[i + 1]
            if nid.startswith('ho'):
                pass
            else:
                grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                grppos = regrip.regg.node[nid]['fgrcenter']
                grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
                grprot = regrip.regg.node[nid]['hndrotmat3np']
                grpjawwidth = regrip.regg.node[nid]['jawwidth']
                # initialize
                armjntsgrpworldaworldz = []
                armjntsgrpworlda = []
                armjntsgrp = []
                armjntsgrphandx = []
                armjntsgrphandxworldz = []
                # choice
                if nid.startswith('rgt'):
                    armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot, armid = 'rgt')
                    armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot, armid = 'rgt')
                    armjntsgrp = regrip.robot.numik(grppos, grprot, armid = 'rgt')
                    armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot, armid = 'rgt')
                    armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot, armid = 'rgt')
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrpworldaworldz, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrpworlda, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrphandx, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrphandxworldz, regrip.robot.initlftjntsr[1:]])
                    jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                if nid.startswith('lft'):
                    armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot, armid = 'lft')
                    armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot, armid = 'lft')
                    armjntsgrp = regrip.robot.numik(grppos, grprot, armid = 'lft')
                    armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot, armid = 'lft')
                    armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot, armid = 'lft')
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
                objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
                objmat4list.append(objmat4worldaworldz)
                objmat4list.append(objmat4worlda)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4handx)
                objmat4list.append(objmat4handxworldz)
        elif i + 1 != len(pathnidlist) - 1:
            # not two node path, middle nodes, if transit, pass
            if regrip.regg.edge[pathnidlist[i]][pathnidlist[i+1]]['edgetype'] == "transit":
                pass
            # if handovertransit
            elif regrip.regg.edge[pathnidlist[i]][pathnidlist[i+1]]['edgetype'] == "handovertransit":
                nid0 = pathnidlist[i]
                nid1 = pathnidlist[i+1]
                #### nid0 move to handover
                grppos0 = regrip.regg.node[nid0]['fgrcenter']
                grprot0 = regrip.regg.node[nid0]['hndrotmat3np']
                grpjawwidth0 = regrip.regg.node[nid0]['jawwidth']
                # initialize
                armjntsgrp0 = []
                # choice
                if nid0.startswith('horgt'):
                    armjntsgrp0 = regrip.robot.numik(grppos0, grprot0, armid = 'rgt')
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp0, regrip.robot.initlftjntsr[1:]])
                    jawwidth.append([grpjawwidth0, regrip.robothand.jawwidthopen])
                elif nid0.startswith('holft'):
                    armjntsgrp0 = regrip.robot.numik(grppos0, grprot0, armid = 'lft')
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp0])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth0])
                objmat40 = regrip.regg.node[nid]['floatingposerotmat4']
                objmat4list.append(objmat40)
                #### nid1 move to handover
                grppos1handx = regrip.regg.node[nid1]['fgrcenterhandx']
                grppos1 = regrip.regg.node[nid1]['fgrcenter']
                grprot1 = regrip.regg.node[nid1]['hndrotmat3np']
                grpjawwidth1 = regrip.regg.node[nid1]['jawwidth']
                # initialize
                armjntsgrp1 = []
                # choice
                if nid1.startswith('horgt'):
                    armjntsgrp1handx = regrip.robot.numik(grppos1handx, grprot1, armid = 'rgt')
                    armjntsgrp1 = regrip.robot.numik(grppos1, grprot1, armid = 'rgt')
                    numikrlist.append([regrip.robot.initrgtjntsr[0], armjntsgrp1handx, armjntsgrp0])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], armjntsgrp1, armjntsgrp0])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], armjntsgrp1, armjntsgrp0])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth0])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth0])
                    jawwidth.append([grpjawwidth1, grpjawwidth0])
                elif nid1.startswith('holft'):
                    armjntsgrp1handx = regrip.robot.numik(grppos1handx, grprot1, armid = 'lft')
                    armjntsgrp1 = regrip.robot.numik(grppos1, grprot1, armid = 'lft')
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp0, armjntsgrp1handx])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp0, armjntsgrp1])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp0, armjntsgrp1])
                    jawwidth.append([grpjawwidth0, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth0, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth0, grpjawwidth1])
                objmat41 = regrip.regg.node[nid1]['floatingposerotmat4']
                objmat4list.append(objmat41)
                objmat4list.append(objmat41)
                objmat4list.append(objmat41)
                #### nid0 move back
                grpposb = regrip.regg.node[nid0]['fgrcenterhandx']
                grprotb = regrip.regg.node[nid0]['hndrotmat3np']
                # initialize
                armjntsgrpb = []
                # choice
                if nid0.startswith('horgt'):
                    armjntsgrpb = regrip.robot.numik(grpposb, grprotb, armid = 'rgt')
                    numikrlist.append([regrip.robot.initrgtjntsr[0], armjntsgrp0, armjntsgrp1])
                    numikrlist.append([regrip.robot.initrgtjntsr[0], armjntsgrpb, armjntsgrp1])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth1])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth1])
                elif nid0.startswith('holft'):
                    armjntsgrpb = regrip.robot.numik(grpposb, grprotb, armid = 'lft')
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp1, armjntsgrp0])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp1, armjntsgrpb])
                    jawwidth.append([grpjawwidth1, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth1, regrip.robothand.jawwidthopen])
                if nid0.startswith('horgt'):
                    numikrlist.append([regrip.robot.initrgtjntsr[0], regrip.robot.initrgtjntsr[1:], armjntsgrp1])
                    jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth1])
                elif nid0.startswith('holft'):
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp1, regrip.robot.initlftjntsr[1:]])
                    jawwidth.append([grpjawwidth1, regrip.robothand.jawwidthopen])
                objmat4b = regrip.regg.node[nid0]['floatingposerotmat4']
                objmat4list.append(objmat4b)
                objmat4list.append(objmat4b)
                objmat4list.append(objmat4b)
            else:
                print regrip.regg.edge[pathnidlist[i]][pathnidlist[i+1]]['edgetype']
                # not two node path, middle nodes, if transfer
                ## middle first
                if nid.startswith('ho'):
                    pass
                else:
                    nid = pathnidlist[i]
                    grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
                    grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                    grppos = regrip.regg.node[nid]['fgrcenter']
                    grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                    grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                    grprot = regrip.regg.node[nid]['hndrotmat3np']
                    grpjawwidth = regrip.regg.node[nid]['jawwidth']
                    # initialize
                    # choice
                    if nid.startswith('rgt'):
                        armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot, armid = 'rgt')
                        armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot, armid = 'rgt')
                        armjntsgrp = regrip.robot.numik(grppos, grprot, armid = 'rgt')
                        armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot, armid = 'rgt')
                        armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot, armid = 'rgt')
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrphandxworldz, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrphandx, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrpworlda, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrpworldaworldz, regrip.robot.initlftjntsr[1:]])
                        jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                        jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                        jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                        jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                        jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                        jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                    if nid.startswith('lft'):
                        armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot, armid = 'lft')
                        armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot, armid = 'lft')
                        armjntsgrp = regrip.robot.numik(grppos, grprot, armid = 'lft')
                        armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot, armid = 'lft')
                        armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot, armid = 'lft')
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
                    objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
                    objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                    objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                    objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                    objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                    objmat4list.append(objmat4handxworldz)
                    objmat4list.append(objmat4handx)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4worlda)
                    objmat4list.append(objmat4worldaworldz)
                ## middle second
                nid = pathnidlist[i + 1]
                # could be ho
                if nid.startswith('ho'):
                    pass
                else:
                    grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                    grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                    grppos = regrip.regg.node[nid]['fgrcenter']
                    grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                    grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
                    grprot = regrip.regg.node[nid]['hndrotmat3np']
                    grpjawwidth = regrip.regg.node[nid]['jawwidth']
                    # initialize
                    # choice
                    if nid.startswith('rgt'):
                        armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot, armid = 'rgt')
                        armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot, armid = 'rgt')
                        armjntsgrp = regrip.robot.numik(grppos, grprot, armid = 'rgt')
                        armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot, armid = 'rgt')
                        armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot, armid = 'rgt')
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrpworldaworldz, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrpworlda, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrphandx, regrip.robot.initlftjntsr[1:]])
                        numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrphandxworldz, regrip.robot.initlftjntsr[1:]])
                        jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                        jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                        jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                        jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                        jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                        jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    if nid.startswith('lft'):
                        armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot, armid = 'lft')
                        armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot, armid = 'lft')
                        armjntsgrp = regrip.robot.numik(grppos, grprot, armid = 'lft')
                        armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot, armid = 'lft')
                        armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot, armid = 'lft')
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
                    objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                    objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                    objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                    objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                    objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
                    objmat4list.append(objmat4worldaworldz)
                    objmat4list.append(objmat4worlda)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4)
                    objmat4list.append(objmat4handx)
                    objmat4list.append(objmat4handxworldz)
        else:
            # not two node path, end nodes, transfer
            ## second to last node
            nid = pathnidlist[i]
            print nid
            if nid.startswith('ho'):
                pass
            else:
                grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
                grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
                grppos = regrip.regg.node[nid]['fgrcenter']
                grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
                grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
                grprot = regrip.regg.node[nid]['hndrotmat3np']
                grpjawwidth = regrip.regg.node[nid]['jawwidth']
                # initialize
                # choice
                if nid.startswith('rgt'):
                    armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot, armid = 'rgt')
                    armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot, armid = 'rgt')
                    armjntsgrp = regrip.robot.numik(grppos, grprot, armid = 'rgt')
                    armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot, armid = 'rgt')
                    armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot, armid = 'rgt')
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrphandxworldz, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrphandx, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrp, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrpworlda, regrip.robot.initlftjntsr[1:]])
                    numikrlist.append([regrip.robot.initlftjntsr[0], armjntsgrpworldaworldz, regrip.robot.initlftjntsr[1:]])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                    jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                if nid.startswith('lft'):
                    armjntsgrphandxworldz = regrip.robot.numik(grpposhandxworldz, grprot, armid = 'lft')
                    armjntsgrphandx = regrip.robot.numik(grpposhandx, grprot, armid = 'lft')
                    armjntsgrp = regrip.robot.numik(grppos, grprot, armid = 'lft')
                    armjntsgrpworlda = regrip.robot.numik(grpposworlda, grprot, armid = 'lft')
                    armjntsgrpworldaworldz = regrip.robot.numik(grpposworldaworldz, grprot, armid = 'lft')
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
                objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
                objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
                objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
                objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
                objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
                objmat4list.append(objmat4handxworldz)
                objmat4list.append(objmat4handx)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4)
                objmat4list.append(objmat4worlda)
                objmat4list.append(objmat4worldaworldz)
            ## last node
            nid = pathnidlist[i + 1]
            grpposworldaworldz = regrip.regg.node[nid]['fgrcenterworldaworldz']
            grpposworlda = regrip.regg.node[nid]['fgrcenterworlda']
            grppos = regrip.regg.node[nid]['fgrcenter']
            grpposhandx = regrip.regg.node[nid]['fgrcenterhandx']
            grpposhandxworldz = regrip.regg.node[nid]['fgrcenterhandxworldz']
            grprot = regrip.regg.node[nid]['hndrotmat3np']
            grpjawwidth = regrip.regg.node[nid]['jawwidth']
            # initialize
            # choice
            if nid.startswith('goalrgt'):
                armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot, armid = 'rgt')
                armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot, armid = 'rgt')
                armjntsgrp = regrip.robot.numikr(grppos, grprot, armid = 'rgt')
                armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot, armid = 'rgt')
                armjntsgrphandxworldz = regrip.robot.numikr(grpposhandxworldz, grprot, armid = 'rgt')
                numikrlist.append([armjntsgrpworldaworldz[0], armjntsgrpworldaworldz[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrpworlda[0], armjntsgrpworlda[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrp[0], armjntsgrp[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrphandx[0], armjntsgrphandx[1], regrip.robot.initlftjnts])
                numikrlist.append([armjntsgrphandxworldz[0], armjntsgrphandxworldz[1], regrip.robot.initlftjnts])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([grpjawwidth, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
            if nid.startswith('goallft'):
                armjntsgrpworldaworldz = regrip.robot.numikr(grpposworldaworldz, grprot, armid = 'lft')
                armjntsgrpworlda = regrip.robot.numikr(grpposworlda, grprot, armid = 'lft')
                armjntsgrp = regrip.robot.numikr(grppos, grprot, armid = 'lft')
                armjntsgrphandx = regrip.robot.numikr(grpposhandx, grprot, armid = 'lft')
                armjntsgrphandxworldz = regrip.robot.numikr(grpposhandxworldz, grprot, armid = 'lft')
                numikrlist.append([armjntsgrpworldaworldz[0], regrip.robot.initrgtjnts, armjntsgrpworldaworldz[1]])
                numikrlist.append([armjntsgrpworlda[0], regrip.robot.initrgtjnts, armjntsgrpworlda[1]])
                numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                numikrlist.append([armjntsgrp[0], regrip.robot.initrgtjnts, armjntsgrp[1]])
                numikrlist.append([armjntsgrphandx[0], regrip.robot.initrgtjnts, armjntsgrphandx[1]])
                numikrlist.append([armjntsgrphandxworldz[0], regrip.robot.initrgtjnts, armjntsgrphandxworldz[1]])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, grpjawwidth])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
                jawwidth.append([regrip.robothand.jawwidthopen, regrip.robothand.jawwidthopen])
            objmat4worldaworldz = regrip.regg.node[nid]['tabletopplacementrotmatworldaworldz']
            objmat4worlda = regrip.regg.node[nid]['tabletopplacementrotmatworlda']
            objmat4 = regrip.regg.node[nid]['tabletopplacementrotmat']
            objmat4handx = regrip.regg.node[nid]['tabletopplacementrotmathandx']
            objmat4handxworldz = regrip.regg.node[nid]['tabletopplacementrotmathandxworldz']
            objmat4list.append(objmat4worldaworldz)
            objmat4list.append(objmat4worlda)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4)
            objmat4list.append(objmat4handx)
            objmat4list.append(objmat4handxworldz)
    return [objmat4list, numikrlist, jawwidth]

if __name__=='__main__':
    gdb = db.GraspDB()
    handpkg = rtq85nm
    nxtrobot = nxt.NxtRobot()
    # handpkg = hrp5threenm
    # hrp5nrobot = hrp5n.Hrp5NRobot()

    base = pandactrl.World(camp=[4000,0,2500], lookatp=[0,0,0])
    # base = pandactrl.World(camp=[300,-2000,1000], lookatp=[300,0,0])

    # ttube.stl
    this_dir, this_filename = os.path.split(__file__)
    # objpath = os.path.join(os.path.split(os.path.split(this_dir)[0])[0], "grip", "objects", "ttube.stl")
    # objpath = os.path.join(os.path.split(os.path.split(this_dir)[0])[0], "grip", "objects", "tool.stl")
    # objpath = os.path.join(os.path.split(os.path.split(this_dir)[0])[0], "grip", "objects", "planewheel.stl")
    # objpath = os.path.join(os.path.split(os.path.split(this_dir)[0])[0], "grip", "objects", "planelowerbody.stl")
    # objpath = os.path.join(os.path.split(os.path.split(this_dir)[0])[0], "grip", "objects", "planefrontstay.stl")
    objpath = os.path.join(os.path.split(os.path.split(this_dir)[0])[0], "grip", "objects", "planerearstay.stl")
    regrip = regriptppfp.RegripTppFp(objpath, nxtrobot, handpkg, gdb, base)

    # ttube
    # startrotmat4 = Mat4(1.0,0.0,0.0,0.0,
    #                     0.0,6.12323426293e-17,-1.0,0.0,
    #                     0.0,1.0,6.12323426293e-17,0.0,
    #                     399.996276855,-100.998413086,-10.9995155334,1.0)
    # goalrotmat4 = Mat4(1.0,0.0,0.0,0.0,
    #                    0.0,6.12323426293e-17,-1.0,0.0,
    #                    0.0,1.0,6.12323426293e-17,0.0,
    #                    499.996276855,200.001571655,-10.9995155334,1.0)
    # goalrotmat4 = Mat4(-0.707106769085,0.707106769085,0.0,0.0,0.707106769085,0.707106769085,-0.0,0.0,0.0,0.0,-1.0,0.0,350.004150391,249.998901367,-16.9616088867,1.0)
    # startrotmat4 = Mat4(0.000547349976841, 2.33689494422e-08, -0.999999821186, 0.0, 2.33689494422e-08, 1.0, 2.33817445405e-08, 0.0, 0.999999821186, -2.33817445405e-08, 0.000547349976841, 0.0, 520.001556396, -249.997833252, -39.9998168945, 1.0)
    #tool
    # startrotmat4 = Mat4(-0.0176398064941,-0.0176398064941,-0.99968880415,0.0,-0.707106769085,0.707106769085,0.0,0.0,0.706886708736,0.706886708736,-0.0249464549124,0.0,225.010162354,100,44.9175643921,1.0)
    # startrotmat4 = Mat4(0.129405856133,0.129405856133,0.98311150074,0.0,0.707106769085,-0.707106769085,0.0,0.0,0.69516479969,0.69516479969,-0.183007523417,0.0,227.126983643,-327.023590088,74.7615509033,1.0)
    # goalrotmat4 = Mat4(-1.0,1.22464685259e-16,0.0,0.0,-1.22464685259e-16,-1.0,0.0,0.0,0.0,0.0,1.0,0.0,294.747955322,-300.0731293559074,-3.99246982852e-06,1.0)
    # planewheel
    # startrotmat4 = Mat4(0.707106769085,0.707106769085,0.0,0.0,-4.32978030171e-17,4.32978030171e-17,-1.0,0.0,-0.707106769085,0.707106769085,6.12323426293e-17,0.0,400.0,-400.0,29.9999980927,1.0)
    # goalrotmat4 = Mat4(0.707106769085,-0.707106769085,0.0,0.0,4.32978030171e-17,4.32978030171e-17,-1.0,0.0,0.707106769085,0.707106769085,6.12323426293e-17,0.0,400.0,-1.7017070846e-15,29.9999980927,1.0)
    # planelowerbody
    # startrotmat4 = Mat4(1.35963113277e-32,6.12323426293e-17,-1.0,0.0,-1.0,2.22044604925e-16,0.0,0.0,2.22044604925e-16,1.0,6.12323426293e-17,0.0,399.997558594,-16.3771038055,74.2884140015,1.0)
    # goalrotmat4 = Mat4(1.35963113277e-32,6.12323426293e-17,-1.0,0.0,-1.0,2.22044604925e-16,0.0,0.0,2.22044604925e-16,1.0,6.12323426293e-17,0.0,399.997558594,-216.377105713,74.2884140015,1.0)
    # planefrontstay
    # startrotmat4 = Mat4(0.707106769085,-0.707106769085,0.0,0.0,0.707106769085,0.707106769085,0.0,0.0,0.0,0.0,1.0,0.0,399.983917236,-399.987426758,8.91688500815e-07,1.0)
    # goalrotmat4 = Mat4(-4.32978030171e-17,-4.32978030171e-17,-1.0,0.0,0.707106769085,-0.707106769085,0.0,0.0,-0.707106769085,-0.707106769085,6.12323426293e-17,0.0,424.570098877,224.573623657,14.9999990463,1.0)
    # goalrotmat4 = Mat4(-6.12323426293e-17,7.49879952079e-33,1.0,0.0,-1.22464685259e-16,-1.0,0.0,0.0,1.0,-1.22464685259e-16,6.12323426293e-17,0.0,465.250152588,200.002487183,15.000002861,1.0)
    # planerearstay
    startrotmat4 = Mat4(0.997635126114,1.59414832979e-06,0.0687321424484,0.0,1.59414832979e-06,1.0,-4.63324249722e-05,0.0,-0.0687321424484,4.63324249722e-05,0.997635126114,0.0,529.004089355,-300.000244141,-43.1730613708,1.0)
    goalrotmat4 = Mat4(-0.319523245096,-0.00040422056918,0.947578370571,0.0,-0.00040422056918,0.999999880791,0.000290279567707,0.0,-0.947578370571,-0.000290279567707,-0.319523364305,0.0,469.728881836,200.048553467,101.955215454,1.0)
    objstart = pg.genObjmnp(objpath, color=Vec4(.3, .0, .0, .1))
    objstart.setMat(startrotmat4)
    objend = pg.genObjmnp(objpath, color=Vec4(.0, .3, .0, .1))
    objend.setMat(goalrotmat4)
    objstart.reparentTo(base.render)
    objend.reparentTo(base.render)
    #
    #
    import time
    tic = time.clock()
    regrip.findshortestpath(startrotmat4, goalrotmat4, base)
    toc = time.clock()
    print toc-tic
    # assert False
    #
    pltfig = plt.figure()
    regrip.plotgraph(pltfig)
    id = 3
    # choice = 'startlftgoallft'
    choice = 'startrgtgoalrgt'
    # choice = 'startlftgoalrgt'
    # choice = 'startrgtgoallft'
    regrip.plotshortestpath(pltfig, id = id, choice = choice)
    plt.axis("equal")
    plt.show()
    # set execute
    import robotcon.nextage as nxtcon
    import robotcon.rtq85 as rtq85con

    nxts = nxtcon.NxtSocket()
    rtq85rs = rtq85con.Rtq85Socket(handname='rgt')
    rtq85ls = rtq85con.Rtq85Socket(handname='lft')
    rtq85rs.initialize()
    rtq85ls.initialize()
    rtq85rs.openhandto(regrip.robothand.jawwidthopen)
    rtq85ls.openhandto(regrip.robothand.jawwidthopen)
    nxts.initialize()
    #
    [objms, numikrms, jawwidth] = getMotionSequence(regrip, id = id, choice = choice)
    nxtmnp = [None]
    objmnp = [None]
    counter = [0]
    def updateshow(objms, numikrms, jawwidth, nxtmnp, objmnp, counter, nxtrobot, handpkg, objpath, task):
        if counter[0] < len(numikrms):
            if nxtmnp[0] is not None:
                nxtmnp[0].detachNode()
            if objmnp[0] is not None:
                objmnp[0].detachNode()
            print counter[0]
            print numikrms[counter[0]]
            nxtrobot.movealljnts([numikrms[counter[0]][0], 0, 0]+numikrms[counter[0]][1].tolist()+numikrms[counter[0]][2].tolist() )
            nxtmnp[0] = nxtplot.genmnp(nxtrobot, handpkg, jawwidthrgt=jawwidth[counter[0]][0], jawwidthlft=jawwidth[counter[0]][1])
            nxtrobot.goinitpose()
            nxtmnp[0].reparentTo(base.render)
            objmnp[0] = pg.genObjmnp(objpath, color = Vec4(.7,0.3,0,1))
            objmnp[0].setMat(objms[counter[0]])
            # pg.plotAxisSelf(base.render,objms[counter[0]].getRow3(3), objms[counter[0]])
            objmnp[0].reparentTo(base.render)
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
    taskMgr.doMethodLater(1, updateshow, "updateshow",
                          extraArgs = [objms, numikrms, jawwidth, nxtmnp, objmnp, counter, nxtrobot, handpkg, objpath],
                          appendTask = True)

    # # one time show for start and end
    # objstart = pg.genObjmnp(objpath, color=Vec4(.7, .7, 0, 1))
    # objstart.setMat(startrotmat4)
    # objend = pg.genObjmnp(objpath, color=Vec4(.3, .3, 0, .5))
    # objend.setMat(goalrotmat4)
    #
    # objstart.reparentTo(base.render)
    # objend.reparentTo(base.render)
    #
    # this_dir, this_filename = os.path.split(__file__)
    # ttpath = Filename.fromOsSpecific(
    #     os.path.join(os.path.split(this_dir)[0] + os.sep, "grip", "supports", "tabletop.egg"))
    # ttnodepath = NodePath("tabletop")
    # ttl = loader.loadModel(ttpath)
    # ttl.instanceTo(ttnodepath)
    # ttnodepath.reparentTo(base.render)

    base.run()