#!/usr/bin/python

"""
The script is written following
http://myenigma.hatenablog.com/entry/2016/03/23/092002
the original file was 2d

# rrt is much faster than rrt connect

author: weiwei
date: 20170609
"""

import random
import copy
import numpy as np


class RRT(object):

    def __init__(self, start, goal, iscollidedfunc, jointlimits, expanddis=.5, goalsamplerate=10, maxiter=5000,
                 robot = None, robotplot = None, handpkg = None):
        """

        :param start: nd point, list
        :param goal: nd point, list
        :param iscollidedfunc: a function returns whether collided or not
        :param jointlimits: [[join0low, joint0high], [joint1low, joint1high], ...]
        :param expandDis: how much to expand along the vector randnode - nearestnode
        :param goalSampleRate: bias to set randnode to be goal
        :param maxIter:
        :param the last three are for robotsim robots

        author: weiwei
        date: 20170609
        """

        self.__start = np.asarray(start)
        self.__end = np.asarray(goal)

        self.__iscollidedcallback = iscollidedfunc
        self.__jointlimits = jointlimits
        self.__expanddis = expanddis
        self.__goalsamplerate = goalsamplerate
        self.__maxiter = maxiter

        self.__robot = robot
        self.__robotplot = robotplot
        self.__handpkg = handpkg

        self.__nodelist = [Node(self.__start)]

    @property
    def start(self):
        # read-only property
        return self.__start

    @property
    def end(self):
        # read-only property
        return self.__end

    @property
    def nodelist(self):
        # read-only property
        return self.__nodelist

    def planning(self, obstaclelist=[], animation=False):
        """
        Pathplanning

        animation: flag for animation on or off

        :return path [[joint0, joint1, ...], [joint0, joint1, ...], ...]
        """

        itercount = 0

        # one sampled point is defined as [point, iscollided]
        sampledpoints = []
        self.__nodelist = [Node(self.__start)]

        while True:

            if itercount > self.__maxiter:
                print "failed to find a path"
                break

            # Random Sampling
            randnode = []
            if random.randint(0, 100) > self.__goalsamplerate:
                for i,jntrng in enumerate(self.__jointlimits):
                    randnode.append(random.uniform(jntrng[0], jntrng[1]))
                randnode = np.asarray(randnode)
            else:
                randnode = copy.deepcopy(self.__end)

            # Find nearest node
            nind = self.getNearestListIndex(self.__nodelist, randnode)
            vec = randnode-self.__nodelist[nind].point
            vec = vec/np.linalg.norm(vec)

            # expand tree
            nearestnode = self.__nodelist[nind]
            newnode = copy.deepcopy(nearestnode)
            newnode.point += self.__expanddis * vec
            newnode.parent = nind

            iscollided = self.__iscollidedcallback(newnode.point, obstaclelist,
                                                   self.__robot, self.__robotplot, self.__handpkg)
            if iscollided:
                sampledpoints.append([newnode.point, True])
                continue

            sampledpoints.append([newnode.point, False])
            self.__nodelist.append(newnode)

            # check goal
            d = np.linalg.norm(newnode.point - self.__end)
            if d <= self.__expanddis:
                print("reaching the goal")
                break

            if animation:
                drawwspace(self, obstaclelist, randnode)

            itercount += 1

        path = [self.__end.tolist()]
        lastindex = len(self.__nodelist) - 1
        # WARNING the first nodelist is a numpyarray
        while self.__nodelist[lastindex].parent is not None:
            node = self.__nodelist[lastindex]
            path.append(node.point.tolist())
            lastindex = node.parent
        path.append(self.__start.tolist())

        return [path, sampledpoints]

    def planningcallback(self, obstaclelist = []):
        """
        decomposed planning
        designed for callbacks of panda3dplot

        :param obstaclelist:
        :return:

        author: weiwei
        date: 20170609
        """

        # Random Sampling
        randnode = []
        if random.randint(0, 100) > self.__goalsamplerate:
            for i,jntrng in enumerate(self.__jointlimits):
                randnode.append(random.uniform(jntrng[0], jntrng[1]))
            randnode = np.asarray(randnode)
        else:
            randnode = copy.deepcopy(self.__end)

        # Find nearest node
        nind = self.getNearestListIndex(self.__nodelist, randnode)
        vec = randnode-self.__nodelist[nind].point
        vec = vec/np.linalg.norm(vec)

        # expand tree
        nearestnode = self.__nodelist[nind]
        newnode = copy.deepcopy(nearestnode)
        newnode.point += self.__expanddis * vec
        newnode.parent = nind
        self.newnode = newnode.point

        iscollided = self.__iscollidedcallback(newnode.point, obstaclelist, self.__robot,
                                               self.__robotplot, self.__handpkg)
        if iscollided:
            return "collided"

        self.__nodelist.append(newnode)

        # check goal
        d = np.linalg.norm(newnode.point - self.__end)
        if d <= self.__expanddis:
            print("reaching the goal")
            return "done"

        return "continue"

    def getpathcallback(self):
        """
        decomposed planning
        designed for callbacks of panda3dplot

        :param obstaclelist:
        :return:

        author: weiwei
        date: 20170609
        """

        path = [self.__end.tolist()]
        lastindex = len(self.__nodelist) - 1
        # WARNING the first nodelist is a numpyarray
        while self.__nodelist[lastindex].parent is not None:
            node = self.__nodelist[lastindex]
            path.append(node.point.tolist())
            lastindex = node.parent
        path.append(self.__start.tolist())

        return path

    def getNearestListIndex(self, nodelist, randnode):
        dlist = [np.linalg.norm(randnode-node.point) for node in nodelist]
        minind = dlist.index(min(dlist))
        return minind

class Node():
    """
    RRT Node
    """

    def __init__(self, point):
        """

        :param point: nd point, numpyarray

        author: weiwei
        date: 20170609
        """

        self.point = point
        self.parent = None

def iscollidedfunc(point, obstacleList, robot = None, robotplot = None, handpkg = None):
    for (obpos, size) in obstacleList:
        d = np.linalg.norm(np.asarray(obpos) - point)
        if d <= size:
            return True  # collision

    return False  # safe


def drawwspace(planner, obstaclelist, randconfiguration=None):
    """
    Draw Graph
    """
    import matplotlib.pyplot as plt
    plt.clf()
    if randconfiguration is not None:
        plt.plot(randconfiguration[0], randconfiguration[1], "^k")
    for node in planner.nodelist:
        if node.parent is not None:
            plt.plot([node.point[0], planner.nodelist[node.parent].point[0]],
                     [node.point[1], planner.nodelist[node.parent].point[1]], "-g")
    plt.plot([point[0] for (point, size) in obstaclelist], [point[1] for (point, size) in obstaclelist], "ok",
             ms=size * 20)
    plt.plot(planner.start[0], planner.start[1], "xr")
    plt.plot(planner.end[0], planner.end[1], "xr")
    plt.axis([-2, 15, -2, 15])
    plt.grid(True)
    plt.pause(0.001)

if __name__ == '__main__':
    import matplotlib.pyplot as plt

    # ====Search Path with RRT====
    obstaclelist = [
        ((5, 5), 1),
        ((3, 6), 2),
        ((3, 8), 2),
        ((3, 10), 2),
        ((7, 5), 2),
        ((9, 5), 2)
    ]  # [x,y,size]
    # Set Initial parameters
    rrt = RRT(start=[0.0, 0.0], goal=[5.0, 10.0], iscollidedfunc = iscollidedfunc,
              jointlimits = [[-2.0, 15.0], [-2.0, 15.0]])

    import time
    tic = time.clock()
    path = rrt.planning(obstaclelist=obstaclelist, animation=True)
    toc = time.clock()
    print toc-tic

    # Draw final path
    drawwspace(rrt, obstaclelist)
    plt.plot([point[0] for point in path], [point[1] for point in path], '-r')
    plt.grid(True)
    plt.pause(0.001)  # Need for Mac
    plt.show()