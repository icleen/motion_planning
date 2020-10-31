#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import numpy as np
import matplotlib.pyplot as plotter
from math import pi
from collisions import PolygonEnvironment
import time
from graph_search import a_star_search

_DEBUG = False

_TRAPPED = 'trapped'
_ADVANCED = 'advanced'
_REACHED = 'reached'

def vdist(veca, vecb):
    return np.sqrt(np.power(veca - vecb, 2).sum())

def get_vec(pt1, pt2, epsilon):
    vec = pt2 - pt1
    norm = np.sqrt(np.power(vec, 2).sum())
    vec = (vec / norm) * epsilon
    vec = pt1 + vec
    return vec

def get_steps(pt1, pt2, epsilon):
    vec = pt2 - pt1
    norm = np.sqrt(np.power(vec, 2).sum())
    normed = (vec / norm) * epsilon
    nsteps = int(norm/epsilon)
    return [pt1 + step*normed for step in range(nsteps)]

class PRM(object):
    """docstring for PRM."""

    def __init__( self, num_samples, num_dimensions=2, step_length=1,
      lims=None, connect_prob=0.05, collision_func=None ):
        super(PRM, self).__init__()
        # same setup
        self.Ts = None
        self.Tg = None

        self.samps = num_samples
        self.dims = num_dimensions
        self.kn = 4
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

        self.build_prm()

    def sample(self):
        rand = np.random.rand(self.samps,self.dims) * self.ranges \
          + self.limits[:,0]
        colis = [pi for pi, pt in enumerate(rand) if self.in_collision(pt)]
        while len(colis) > 0:
            rand[colis] = np.random.rand(len(colis),self.dims) * self.ranges \
              + self.limits[:,0]
            colis = [pi for pi, pt in enumerate(rand) if self.in_collision(pt)]
        return rand

    def nearest_k(self, pt, pts, k):
        dists = np.sqrt(np.power(pts-pt, 2).sum(1))
        return dists.argsort()[:k]

    def connectable(self, pt, pk):
        '''use line connection to see if you can connect 2 points'''
        lpts = get_steps(pt, pk, self.epsilon)
        for lp in lpts:
            if self.in_collision(lp):
                return False
        return True

    def build_prm(self):
        '''
        Build the prm map
        '''
        pts = self.sample()
        self.states = pts
        self.map = np.zeros((pts.shape[0], pts.shape[0]))

        for pi, pt in enumerate(pts):
            ptks = self.nearest_k(pt, pts[np.r_[:pi, pi+1:len(pts)]], self.kn)
            ptks[ptks>=pi] += 1
            for pk in ptks:
                if pk == pi:
                    import pdb; pdb.set_trace()
                if self.connectable(pt, pts[pk]):
                    self.map[pi, pk] = 1

    def plan(self, init, goal):
        '''
        Build the rrt connect from init to goal
        Returns path to goal or None
        '''
        self.init = np.array(init)
        self.goal = np.array(goal)

        inear = 0
        ptks = self.nearest_k(self.init, self.states, self.kn)
        for pk in ptks:
            if self.connectable(self.init, self.states[pk]):
                inear = pk
                break
        gnear = 0
        ptks = self.nearest_k(self.goal, self.states, self.kn)
        for pk in ptks:
            if self.connectable(self.goal, self.states[pk]):
                gnear = pk
                break

        def transition(state, action):
            if action > self.kn:
                import pdb; pdb.set_trace()
            return np.where(self.map[state]==1)[0][action]

        def is_goal(state):
            return state == gnear

        def cost(action, state):
            ind = transition(state, action)
            return vdist(self.states[state], self.states[ind])

        def heur(state):
            return vdist(self.states[state], self.states[gnear])

        # ((path, actions), visited)
        path, visited = a_star_search(inear, transition, is_goal, np.arange(self.kn), cost, heur)
        path, actions = path
        if len(path) == 0:
            return None
        return [self.init] + [self.states[pi] for pi in path] + [self.goal]

    def get_states_and_edges(self):
        edges = np.where(self.map == 1)
        edges = [ (self.states[p1], self.states[p2])
          for p1, p2 in zip(edges[0], edges[1]) ]
        return self.states, edges


if __name__ == '__main__':
    PRM()
