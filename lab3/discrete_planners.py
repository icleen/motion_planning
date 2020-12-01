
import sys
import numpy as np
import heapq

from utils import *
from graph_map import GridMap

acost = {
  'u':1,'d':1,'l':1,'r':1,
  'ne':1.5,'nw':1.5,'sw':1.5,'se':1.5,
  'rl':0.25,'rr':0.25
}

def cost(action, state=None):
    if action == 'f':
        if (state[_T] % 2) == 0:
            return 1
        else:
            return 1.5
    else:
        return acost[action]

def dfs(init_state, f, is_goal, actions):
    '''
    Perform depth first search on a grid map.

    init_state - the intial state on the map
    f - transition function of the form s_prime = f(s,a)
    is_goal - function taking as input a state s and returning True if its a goal state
    actions - set of actions which can be taken by the agent

    returns - ((path, action_path), visited) or None if no path can be found
    path - a list of tuples. The first element is the initial state followed by all states
        traversed until the final goal state
    action_path - the actions taken to transition from the initial state to goal state
    '''
    frontier = []
    n0 = SearchNode(init_state, actions)
    visited = []
    frontier.append(n0)
    while len(frontier) > 0:
        # Pop last element
        n_i = frontier.pop()
        if n_i.state not in visited:
            visited.append(n_i.state)
            if is_goal(n_i.state):
                return (backpath(n_i), visited)
            else:
                for a in actions:
                    s_prime = f(n_i.state, a)
                    n_prime = SearchNode(s_prime, actions, n_i, a)
                    frontier.append(n_prime)
    return (([], []), visited)

def dfs_idfs(init_state, f, is_goal, actions, maxdepth):
    '''
    Perform depth first search on a grid map.

    init_state - the intial state on the map
    f - transition function of the form s_prime = f(s,a)
    is_goal - function taking as input a state s and returning True if its a goal state
    actions - set of actions which can be taken by the agent

    returns - ((path, action_path), visited) or None if no path can be found
    path - a list of tuples. The first element is the initial state followed by all states
        traversed until the final goal state
    action_path - the actions taken to transition from the initial state to goal state
    '''
    frontier = []
    n0 = SearchNode(init_state, actions)
    visited = []
    frontier.append((n0, 0))
    while len(frontier) > 0:
        # Pop last element
        n_i, depth = frontier.pop()
        if depth >= maxdepth:
            continue
        if (n_i.state, depth) not in visited:
            visited.append((n_i.state, depth))
            if is_goal(n_i.state):
                return (backpath(n_i), [v for (v,d) in visited])
            else:
                for a in actions:
                    s_prime = f(n_i.state, a)
                    n_prime = SearchNode(s_prime, actions, n_i, a)
                    frontier.append((n_prime, depth+1))
    return (None, [v for (v,d) in visited])

def idfs(init_state, f, is_goal, actions, maxdepth=1000):
    visited = []
    for depth in range(0, maxdepth):
        solution, visited = dfs_idfs(init_state, f, is_goal, actions, maxdepth=depth)
        if solution is not None:
            return (solution, visited)
    return (([], []), visited)

def bfs(init_state, f, is_goal, actions):
    '''
    Perform breadth first search on a grid map.

    init_state - the intial state on the map
    f - transition function of the form s_prime = f(s,a)
    is_goal - function taking as input a state s and returning True if its a goal state
    actions - set of actions which can be taken by the agent

    returns - ((path, action_path), visited) or None if no path can be found
    path - a list of tuples. The first element is the initial state followed by all states
        traversed until the final goal state
    action_path - the actions taken to transition from the initial state to goal state
    '''
    frontier = []
    n0 = SearchNode(init_state, actions)
    visited = []
    frontier.append(n0)
    while len(frontier) > 0:
        # Pop last element
        n_i = frontier.pop(0)
        if n_i.state not in visited:
            visited.append(n_i.state)
            if is_goal(n_i.state):
                return (backpath(n_i), visited)
            else:
                for act in actions:
                    s_prime = f(n_i.state, act)
                    n_prime = SearchNode(s_prime, actions, n_i, act)
                    frontier.append(n_prime)
    return (([], []), visited)

def uniform_cost_search(init_state, f, is_goal, actions):
    frontier = PriorityQ()
    n0 = SearchNode(init_state, actions)
    visited = []
    frontier.push(n0, n0.cost)
    while len(frontier) > 0:
        # Pop last element
        n_i = frontier.pop()
        if n_i.state not in visited:
            visited.append(n_i.state)
            if is_goal(n_i.state):
                return (backpath(n_i), visited)
            else:
                for act in actions:
                    s_prime = f(n_i.state, act)
                    n_prime = SearchNode( s_prime, actions, n_i, act,
                      cost=n_i.cost+cost(act, n_i.state) )
                    if n_prime not in frontier or \
                      (n_prime in frontier and \
                       n_prime.cost < frontier.get_cost(n_prime)):
                        frontier.push(n_prime, n_prime.cost)

    return (([], []), visited)

def a_star_search(init_state, f, is_goal, actions, h):
    '''
    init_state - value of the initial state
    f - transition function takes input state (s), action (a), returns s_prime = f(s, a)
        returns s if action is not valid
    is_goal - takes state as input returns true if it is a goal state
    actions - list of actions available
    h - heuristic function, takes input s and returns estimated cost to goal
        (note h will also need access to the map, so should be a member function of GridMap)
    '''
    frontier = PriorityQ()
    n0 = SearchNode(init_state, actions)
    visited = []
    frontier.push(n0, n0.cost)
    while len(frontier) > 0:
        # Pop last element
        n_i = frontier.pop()
        if n_i.state not in visited:
            visited.append(n_i.state)
            if is_goal(n_i.state):
                return (backpath(n_i), visited)
            else:
                for a in actions:
                    s_prime = f(n_i.state, a)
                    n_prime = SearchNode( s_prime, actions, n_i, a,
                      cost=n_i.cost+cost(a, n_i.state) )
                    hcost = n_prime.cost + h(n_prime.state)
                    if n_prime not in frontier or \
                      (n_prime in frontier and \
                       hcost < frontier.get_cost(n_prime)):
                        frontier.push(n_prime, hcost)

    return (([], []), visited)

def backpath(node):
    '''
    Function to determine the path that lead to the specified search node

    node - the SearchNode that is the end of the path

    returns - a tuple containing (path, action_path) which are lists respectively of the states
    visited from init to goal (inclusive) and the actions taken to make those transitions.
    '''
    # print('cost:', node.cost)
    path = [node.state]
    action_path = []
    while node.parent is not None:
        action_path.append(node.parent_action)
        node = node.parent
        path.append(node.state)
    path.reverse()
    action_path.reverse()
    return (path, action_path)
