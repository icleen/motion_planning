#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import sys
import numpy as np
import heapq
import matplotlib.pyplot as plotter
from math import hypot, sqrt

class GridMap:
    '''
    Class to hold a grid map for navigation. Reads in a map.txt file of the format
    0 - free cell, x - occupied cell, g - goal location, i - initial location.
    Additionally provides a simple transition model for grid maps and a convience function
    for displaying maps.
    '''

    _DEBUG = False
    _DEBUG_END = True
    _ACTIONS = ['u','d','l','r']
    _ACTIONS_2 = ['u','d','l','r','ne','nw','sw','se']
    _ACTIONS_3 = ['f','rl','rr']
    pose2act = {0:'u',1:'ne',2:'r',3:'se',4:'d',5:'sw',6:'l',7:'nw'}

    _T = 2
    _X = 1
    _Y = 0
    _GOAL_COLOR = 0.75
    _INIT_COLOR = 0.25
    _PATH_COLOR_RANGE = 0.5
    _VISITED_COLOR = 0.9

    def __init__(self, map_path=None):
        '''
        Constructor. Makes the necessary class variables. Optionally reads in a provided map
        file given by map_path.

        map_path (optional) - a string of the path to the file on disk
        '''
        self.rows = None
        self.cols = None
        self.goal = None
        self.init_pos = None
        self.occupancy_grid = None
        if map_path is not None:
            self.read_map(map_path)

    def read_map(self, map_path):
        '''
        Read in a specified map file of the format described in the class doc string.

        map_path - a string of the path to the file on disk
        '''
        map_file = open(map_path,'r')
        lines = [l.rstrip().lower() for l in map_file.readlines()]
        map_file.close()
        self.rows = len(lines)
        self.cols = max([len(l) for l in lines])
        if self._DEBUG:
            print('rows', self.rows)
            print('cols', self.cols)
            print(lines)
        self.occupancy_grid = np.zeros((self.rows, self.cols), dtype=np.bool)
        self.states = []
        for r in range(self.rows):
            for c in range(self.cols):
                if lines[r][c] == 'x':
                    self.occupancy_grid[r][c] = True
                self.states.append( (r, c) )
                if lines[r][c] == 'g':
                    self.goal = (r,c)
                elif lines[r][c] == 'i':
                    self.init_pos = (r,c,0)

    def get_states(self):
        return self.states

    def is_goal(self,s):
        '''
        Test if a specifid state is the goal state

        s - tuple describing the state as (row, col) position on the grid.

        Returns - True if s is the goal. False otherwise.
        '''
        return (s[self._X] == self.goal[self._X] and
                s[self._Y] == self.goal[self._Y])

    def transition(self, s, act):
        '''
        Transition function for the current grid map.

        s - tuple describing the state as (row, col) position on the grid.
        a - the action to be performed from state s

        returns - s_prime, the state transitioned to by taking action a in state s.
        If the action is not valid (e.g. moves off the grid or into an obstacle)
        returns the current state.
        '''

        new_pos = list(s[:])
        if act == 'f':
            act = pose2act[new_pos[self._T]]
        # Ensure action stays on the board
        if act == 'rl':
            new_pos[self._T] = (new_pos[self._T] - 1) % 8
        elif act == 'rr':
            new_pos[self._T] = (new_pos[self._T] + 1) % 8
        elif act == 'u':
            if s[self._Y] > 0:
                new_pos[self._Y] -= 1
        elif act == 'd':
            if s[self._Y] < self.rows - 1:
                new_pos[self._Y] += 1
        elif act == 'l':
            if s[self._X] > 0:
                new_pos[self._X] -= 1
        elif act == 'r':
            if s[self._X] < self.cols - 1:
                new_pos[self._X] += 1
        elif act == 'nw':
            if s[self._Y] > 0 and s[self._X] > 0:
                new_pos[self._Y] -= 1
                new_pos[self._X] -= 1
        elif act == 'ne':
            if s[self._Y] > 0 and s[self._X] < self.cols - 1:
                new_pos[self._Y] -= 1
                new_pos[self._X] += 1
        elif act == 'sw':
            if s[self._Y] < self.rows - 1 and s[self._X] > 0:
                new_pos[self._Y] += 1
                new_pos[self._X] -= 1
        elif act == 'se':
            if s[self._Y] < self.rows - 1 and s[self._X] < self.cols - 1:
                new_pos[self._Y] += 1
                new_pos[self._X] += 1
        else:
            print('Unknown action:', str(a))

        # Test if new position is clear
        if self.occupancy_grid[new_pos[0], new_pos[1]]:
            s_prime = tuple(s)
        else:
            s_prime = tuple(new_pos)
        return s_prime

    def display_map(self, path=[], visited={}, filename=None, plot_show=False):
        '''
        Visualize the map read in. Optionally display the resulting plan and visisted nodes

        path - a list of tuples describing the path take from init to goal
        visited - a set of tuples describing the states visited during a search
        filename - relative path to file where image will be saved
        '''
        plotter.clf()
        display_grid = np.array(self.occupancy_grid, dtype=np.float32)

        # Color all visited nodes if requested
        for v in visited:
            display_grid[v[0],v[1]] = self._VISITED_COLOR
        # Color path in increasing color from init to goal
        for i, p in enumerate(path):
            disp_col = self._INIT_COLOR + self._PATH_COLOR_RANGE*(i+1)/len(path)
            display_grid[p[0],p[1]] = disp_col

        display_grid[self.init_pos[0],self.init_pos[1]] = self._INIT_COLOR
        display_grid[self.goal] = self._GOAL_COLOR

        if (path[-1][0], path[-1][1]) == self.goal:
            print('finished')
            # print('finished:', (path[-1][0], path[-1][1]), ', goal:', self.goal)
            plotter.title('finished')

        # Plot display grid for visualization
        imgplot = plotter.imshow(display_grid)
        # Set interpolation to nearest to create sharp boundaries
        imgplot.set_interpolation('nearest')
        # Set color map to diverging style for contrast
        imgplot.set_cmap('Spectral')
        if filename is not None:
            plotter.savefig(filename)
        if plot_show:
            plotter.show()

    def uninformed_heuristic(self, s):
        '''
        Example of how a heuristic may be provided. This one is admissable, but dumb.

        s - tuple describing the state as (row, col) position on the grid.

        returns - floating point estimate of the cost to the goal from state s
        '''
        return 0.0

    def euclidean_heuristic(self, s):
        '''
        Example of how a heuristic may be provided. This one is admissable, but dumb.

        s - tuple describing the state as (row, col) position on the grid.

        returns - floating point estimate of the cost to the goal from state s
        '''
        xdist = s[self._X] - self.goal[self._X]
        ydist = s[self._Y] - self.goal[self._Y]
        return sqrt(xdist**2 + ydist**2)

    def manhattan_heuristic(self, s):
        '''
        Example of how a heuristic may be provided. This one is admissable, but dumb.

        s - tuple describing the state as (row, col) position on the grid.

        returns - floating point estimate of the cost to the goal from state s
        '''
        xdist = s[self._X] - self.goal[self._X]
        ydist = s[self._Y] - self.goal[self._Y]
        return abs(xdist) + abs(ydist)
