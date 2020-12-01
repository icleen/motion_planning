
import os, sys, json, argparse
import os.path as osp
import numpy as np
import matplotlib.pyplot as plt

from rrt import RRT, BidirectionalRRT
from prm import PRM
from vrepWrapper import VrepWrapper

def get_args():
    parser = argparse.ArgumentParser(
      description='Find path through sample based planning',
      formatter_class=argparse.ArgumentDefaultsHelpFormatter )
    parser.add_argument( '-m', '--method', type=str,
      default='rrt',
      help='choose method: [rrt, brrt, prm, afp]' )
    parser.add_argument( '-k', '--num_samples', type=int,
      nargs='?', default=5000,
      help='number of samples' )
    parser.add_argument( '-p', '--connect_prob', type=float,
      nargs='?', default=0.1,
      help='number of samples' )
    parser.add_argument( '-s', '--step', type=float,
      nargs='?', default=0.1,
      help='step length' )
    parser.add_argument( '-g', '--goalpos', type=int,
      nargs='?', default=2,
      help='which goal to use' )
    parser.add_argument( '-pm', '--problem', type=str,
      nargs='?', default='toymap.json',
      help='what problem to test on' )
    return parser.parse_args()

def main():

    config = get_args()

    environment = VrepWrapper(False, None, goalpos=config.goalpos)
    num_samples = config.num_samples
    step_size = config.step

    if config.method == 'rrt':
        rrt = RRT(num_samples,
                  2,
                  step_size,
                  lims=environment.lims,
                  connect_prob=config.connect_prob,
                  collision_func=environment.test_collisions)
        plan = rrt.build_rrt_connect(environment.start, environment.goal)
        environment.draw_plan(plan, rrt.get_states_and_edges())
    elif config.method == 'brrt':
        rrt = BidirectionalRRT(num_samples,
                  2,
                  step_size,
                  lims=environment.lims,
                  connect_prob=config.connect_prob,
                  collision_func=environment.test_collisions)
        plan = rrt.build_rrt_connect(environment.start, environment.goal)
        environment.draw_plan(plan, rrt.get_states_and_edges())
    elif config.method == 'prm':
        prm = PRM(num_samples,
                  2,
                  step_size,
                  lims=environment.lims,
                  connect_prob=0.5,
                  collision_func=environment.test_collisions)
        prm.build_prm()
        plan = prm.plan(environment.start, environment.goal)
        environment.draw_plan(plan, prm.get_states_and_edges())


if __name__ == '__main__':
    main()
