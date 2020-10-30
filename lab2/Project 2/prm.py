#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import numpy as np
import matplotlib.pyplot as plotter
from math import pi
from collisions import PolygonEnvironment
import time

_DEBUG = False

_TRAPPED = 'trapped'
_ADVANCED = 'advanced'
_REACHED = 'reached'

def vdist(veca, vecb):
    return np.sqrt(np.power(veca - vecb, 2).sum())


class PRM(object):
    """docstring for PRM."""

    def __init__( self, num_samples, num_dimensions=2, step_length=1,
      lims=None, connect_prob=0.05, collision_func=None ):
        super(PRM, self).__init__()
        # same setup
        self.Ts = None
        self.Tg = None

        self.K = num_samples
        self.n = num_dimensions
        self.epsilon = step_length
        self.connect_prob = connect_prob

        self.in_collision = collision_func
        if collision_func is None:
            self.in_collision = self.fake_in_collision

        # Setup range limits
        self.limits = lims
        if self.limits is None:
            self.limits = []
            for n in range(num_dimensions):
                self.limits.append([0,100])
            self.limits = np.array(self.limits)

        self.ranges = self.limits[:,1] - self.limits[:,0]
        self.found_path = False

    def build_rrt(self, init, goal):
        '''
        Build the rrt from init to goal
        Returns path to goal or None
        '''
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False

        # Build tree and search
        self.Ts = [RRTSearchTree(init), RRTSearchTree(goal)]

        for k in range(self.K):
            ti = k%2
            self.T = self.Ts[ti]
            rndst = self.sample(self.init if ti else self.goal)
            status, node0 = self.extend(rndst)
            if status != _TRAPPED:
                self.T = self.Ts[ti-1]
                status, node1 = self.extend(node0.state)
                if status == _REACHED:
                    if ti == 1:
                        path0 = self.Ts[0].get_back_path(node1)
                        path1 = self.Ts[1].get_back_path(node0)
                    else:
                        path0 = self.Ts[0].get_back_path(node0)
                        path1 = self.Ts[1].get_back_path(node1)
                    path1.reverse()
                    return path0 + path1[1:]
        return None

    def build_rrt_connect(self, init, goal):
        '''
        Build the rrt connect from init to goal
        Returns path to goal or None
        '''
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False

        # Build tree and search
        self.Ts = [RRTSearchTree(init), RRTSearchTree(goal)]

        for k in range(self.K):
            ti = k%2
            self.T = self.Ts[ti]
            rndst = self.sample(self.init if ti else self.goal)
            status, node0 = self.extend(rndst)
            if status != _TRAPPED:
                self.T = self.Ts[ti-1]
                status, node1 = _ADVANCED, None
                while status == _ADVANCED:
                    status, node1 = self.extend(node0.state, node1)
                    if status == _REACHED:
                        if ti == 1:
                            path0 = self.Ts[0].get_back_path(node1)
                            path1 = self.Ts[1].get_back_path(node0)
                        else:
                            path0 = self.Ts[0].get_back_path(node0)
                            path1 = self.Ts[1].get_back_path(node1)
                        path1.reverse()
                        return path0 + path1[1:]
        return None

    def get_states_and_edges(self):
        states0, edges0 = self.Ts[0].get_states_and_edges()
        states1, edges1 = self.Ts[1].get_states_and_edges()
        states = np.concatenate((states0, states1), 0)
        return states, edges0+edges1
