
import sys
import numpy as np
from collections import defaultdict

from utils import *
from graph_map import GridMap

def value_iter(map, discount=0.8, threshold=0.1, max_iterations=1000):

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
                for (statep, prob) in map.transition(state, action):
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
    return values
