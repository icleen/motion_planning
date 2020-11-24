# -*- coding: utf-8 -*-
"""
Created on Thu Aug 29 12:17:29 2019

@author: tabor
"""
import argparse
import numpy as np
import matplotlib.pyplot as plotter
from math import pi
from collisions import PolygonEnvironment
import time
import vrepWrapper
from rrt import *
from prm import *


def get_args():
    parser = argparse.ArgumentParser(
      description='Find path through sample based planning',
      formatter_class=argparse.ArgumentDefaultsHelpFormatter )
    parser.add_argument( '-c', '--connect', type=bool,
      nargs='?', default=False, const=True,
      help='connect algorithm or regular RRT' )
    parser.add_argument( '-b', '--bidirection', type=bool,
      nargs='?', default=False, const=True,
      help='bidirectional RRT or not' )
    parser.add_argument( '-prm', '--prm', type=bool,
      nargs='?', default=False, const=True,
      help='use prm' )
    parser.add_argument( '-k', '--num_samples', type=int,
      nargs='?', default=5000,
      help='number of samples' )
    parser.add_argument( '-p', '--connect_prob', type=float,
      nargs='?', default=0.05,
      help='number of samples' )
    parser.add_argument( '-s', '--step', type=float,
      nargs='?', default=None,
      help='step length' )
    parser.add_argument( '-m', '--problem', type=str,
      nargs='?', default='./env0.txt',
      help='what problem to test on [./env0.txt, ./env1.txt, vrep]' )
    return parser.parse_args()


def main():
    config = get_args()

    sh = True # only Linux, turn off if not working

    np.random.seed(0)

    #load problem
    if(config.problem == "vrep"):
        try:
            import SharedArray as sa
        except:
            print('switching to not using shared memory')
            sa = None
            sh = False
        environment = vrepWrapper.vrepWrapper(sh,sa)
        step_length=0.1

    elif 'env0' in config.problem:
        environment = PolygonEnvironment()
        environment.read_env(config.problem)
        step_length=2
    elif 'env1' in config.problem:
        environment = PolygonEnvironment()
        environment.read_env(config.problem)
        step_length=0.15

    if config.step is None:
        config.step = step_length


    dims = len(environment.start)
    print(config)
    start_time = time.time()

    if config.prm:
        if config.num_samples == 5000:
            config.num_samples = 1000
        rrt = PRM( config.num_samples,
                   dims,
                   config.step,
                   lims=environment.lims,
                   connect_prob=config.connect_prob,
                   collision_func=environment.test_collisions )
    elif config.bidirection:
        rrt = BidirectionalRRT(config.num_samples,
                               dims,
                               config.step,
                               lims=environment.lims,
                               connect_prob=config.connect_prob,
                               collision_func=environment.test_collisions)
    else:
        rrt = RRT(config.num_samples,
                  dims,
                  config.step,
                  lims=environment.lims,
                  connect_prob=config.connect_prob,
                  collision_func=environment.test_collisions)

    if config.prm:
        plan = rrt.plan(environment.start, environment.goal)
    elif config.connect:
        plan = rrt.build_rrt_connect(environment.start, environment.goal)
    else:
        plan = rrt.build_rrt(environment.start, environment.goal)

    # if(config.problem == "vrep"):
    #    environment.vrepReset()

    run_time = time.time() - start_time
    # print 'plan:', plan
    print('run_time =', run_time)

    save = config.problem.split('/')[-1].split('.')[0]
    save = '{}_{}_{}'.format( save,
      'b' if config.bidirection else 'o',
      'c' if config.connect else 'o' )
    debugThing = environment.draw_plan(plan, rrt, False, True, True, save=save)


    if(config.problem == "vrep"):
        time.sleep(10)
        environment.vrepStop()

if __name__ == '__main__':
    main()
