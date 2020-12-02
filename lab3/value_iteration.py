
import sys
import numpy as np
from collections import defaultdict

from utils import *
from graph_map import GridMap

def value_iter(map, threshold=0.1, max_iterations=1000):
    discount = map.discount
    values = defaultdict(int)
    for iter in range(max_iterations):
        newvalues = defaultdict(int)
        diff = 0.0
        for state in map.get_states():
            if map.is_goal(state):
                continue
            best_aval = float("-inf")
            for action in map.get_actions():
                aval = 0
                for (prob, statep) in map.transition(state, action):
                    reward = map.value(state, action, statep)
                    val = 0 if map.is_goal(statep) else values[statep]
                    aval += prob * (reward + discount * val)
                if aval > best_aval:
                    best_aval = aval
            newvalues[state] = best_aval
            diff += abs(values[state] - newvalues[state])
        values = newvalues
        if diff < threshold:
            break
    policy = value_policy(map, values, threshold, max_iterations)
    path = policy_path(map, policy)
    return values, policy, path, iter

def value_policy(map, values, threshold=0.1, max_iterations=1000):
    discount = map.discount
    policy = {}
    for state in map.get_states():
        best_aval = float("-inf")
        best_act = None
        for action in map.get_actions():
            aval = 0
            for (prob, statep) in map.transition(state, action):
                reward = map.value(state, action, statep)
                val = 0 if map.is_goal(statep) else values[statep]
                aval += prob * (reward + discount * val)
            if aval > best_aval:
                best_aval = aval
                best_act = action
        policy[state] = best_act
    return policy

def policy_path(map, policy):
    state = (map.init_pos[0], map.init_pos[1])
    path = [state]
    iter = 0
    while not map.is_goal(state):
        action = policy[state]
        state = map.transition(state, action, simode=True)
        path.append(state)
        iter += 1
        if iter > 1000:
            break
    return path
