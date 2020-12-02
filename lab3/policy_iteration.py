
import sys
import numpy as np
from collections import defaultdict

from utils import *
from graph_map import GridMap
from value_iteration import policy_path

def policy_iteration(map, threshold=0.1, max_iterations=100):
    discount = map.discount
    policy = defaultdict(lambda: 'u')
    values = defaultdict(lambda: 0)
    iters = 0
    changes = 1
    for iters in range(max_iterations):
        values, viters = policy_evaluation(map, policy, values, threshold, max_iterations)
        policy, changes = policy_improvement(map, policy, values)
        iters += 1
        if iters%10 == 0:
            import pdb; pdb.set_trace()
            print('iters:', iters)
        if changes < 1:
            break
    path = policy_path(map, policy)
    return values, policy, path, iters

def policy_evaluation(map, policy, values, threshold=0.1, max_iterations=100):
    discount = map.discount
    for iter in range(max_iterations):
        newvalues = defaultdict(int)
        diff = 0.0
        for state in map.get_states():
            if map.is_goal(state):
                continue
            action = policy[state]
            aval = 0
            for (prob, statep) in map.transition(state, action):
                reward = map.value(state, action, statep)
                val = 0 if map.is_goal(statep) else values[statep]
                aval += prob * (reward + discount * val)
            newvalues[state] = aval
            diff += abs(values[state] - newvalues[state])
        values = newvalues
        if diff < threshold:
            break
    return values, iter

def policy_improvement(map, policy, values):
    discount = map.discount
    newpolicy = {}
    changes = 0
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
        newpolicy[state] = best_act
        if policy[state] != best_act:
            changes += 1
    return newpolicy, changes
