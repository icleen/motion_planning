
import os, sys, json, argparse
import os.path as osp
import numpy as np
import matplotlib.pyplot as plt

from rrt import RRT, BidirectionalRRT
from prm import PRM
from vrepWrapper import VrepWrapper

def main():
    environment = VrepWrapper(False, None)
    num_samples = 5000
    step_size = 0.1

    rrt = RRT(num_samples,
              2,
              step_size,
              lims=environment.lims,
              connect_prob=0.5,
              collision_func=environment.test_collisions)

    plan = rrt.build_rrt_connect(environment.start, environment.goal)

    environment.draw_plan(plan, rrt.get_states_and_edges())


if __name__ == '__main__':
    main()
