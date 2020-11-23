
import os, sys, json, argparse
import os.path as osp
import numpy as np
import matplotlib.pyplot as plt

from rrt import RRT, BidirectionalRRT
from prm import PRM
from toy_env import draw_map

class Solver(object):
    """docstring for Solver."""

    def __init__(self, ptcloud, threshold=0.75):
        super(Solver, self).__init__()
        self.ptcloud = ptcloud
        self.threshold = threshold

    def iscollision(self, pt):
        dists = np.sqrt(np.power(self.ptcloud - pt, 2).sum(1))
        return np.min(dists) < self.threshold

    def solve(self, position):
        raise NotImplementedError()

class RRT_Solver(Solver):
    """docstring for RRT_Solver."""

    def __init__(self, ptcloud, threshold=0.75,
      samples=5000, connect_prob=0.001, step_length=0.5):
        super(RRT_Solver, self).__init__(ptcloud, threshold)

        lims = np.array([[-18, 17], [-6, 31]])
        self.method = RRT(
          samples, num_dimensions=2, step_length=1,
          lims=lims, connect_prob=connect_prob, collision_func=self.iscollision
        )

    def solve(self, position, goal):
        return self.method.build_rrt_connect(position, goal)

class BRRT_Solver(Solver):
    """docstring for BRRT_Solver."""

    def __init__(self, ptcloud, threshold=0.75,
      samples=50000, connect_prob=0.001, step_length=0.5):
        super(BRRT_Solver, self).__init__(ptcloud, threshold)

        lims = np.array([[-18, 17], [-6, 31]])
        self.method = BidirectionalRRT(
          samples, num_dimensions=2, step_length=1,
          lims=lims, connect_prob=connect_prob, collision_func=self.iscollision
        )

    def solve(self, position, goal):
        return self.method.build_rrt_connect(position, goal)


class PRM_Solver(Solver):
    """docstring for PRM_Solver."""

    def __init__(self, ptcloud, threshold=0.75, samples=5000, connect_prob=0.001, step_length=0.5, connectn=5):
        super(PRM_Solver, self).__init__(ptcloud, threshold)

        lims = np.array([[-18, 17], [-6, 31]])
        self.method = PRM(
          samples, num_dimensions=2, step_length=step_length,
          lims=lims, connect_prob=connect_prob, collision_func=self.iscollision,
          connectn=connectn
        )
        self.method.sample = lambda : self.sample(lims, step_length)
        self.method.build_prm()

    def sample(self, lims, step_length):
        pts = []
        for xi in np.arange(lims[0,0], lims[0,1], 1.2):
            for yi in np.arange(lims[1,0], lims[1,1], 1.2):
                pts.append([xi, yi])
        pts = np.array(pts)
        return pts

    def solve(self, position, goal):
        return self.method.plan(position, goal)

class APF_Solver(Solver):
    """docstring for APF_Solver."""

    def __init__(self, arg):
        super(APF_Solver, self).__init__()
        self.arg = arg


def draw_path(map, start, goal, solver, path=None, wfil='map_path.png'):
    plt.figure(figsize=(5,5))

    Qs, edges = solver.method.get_states_and_edges()
    print('states:', len(Qs))
    # Draw tree for each of the robot links
    for i, e in enumerate(edges):
        e0 = e[0]
        e1 = e[1]
        plt.plot([e0[0], e1[0]], [e0[1], e1[1]], 'b', zorder=-1)
        plt.plot([e0[0], e1[0]], [e0[1], e1[1]], 'b.', zorder=-1)

    if path is not None:
        path = np.array(path)
        plt.plot(path[:,0], path[:,1], color='purple')

    plt.scatter(map[:,0], map[:,1])
    plt.scatter([start[0]], [start[1]], label='start', color='r')
    plt.scatter([goal[0]], [goal[1]], label='goal', color='g')
    plt.legend(loc='lower right')
    plt.savefig(wfil)


def get_args():
    parser = argparse.ArgumentParser(
      description='Find path through sample based planning',
      formatter_class=argparse.ArgumentDefaultsHelpFormatter )
    parser.add_argument( '-m', '--method', type=str,
      default='rrt',
      help='choose method: [rrt, brrt, prm, afp]' )
    parser.add_argument( '-k', '--num_samples', type=int,
      nargs='?', default=10000,
      help='number of samples' )
    parser.add_argument( '-p', '--connect_prob', type=float,
      nargs='?', default=0.01,
      help='number of samples' )
    parser.add_argument( '-s', '--step', type=float,
      nargs='?', default=1,
      help='step length' )
    parser.add_argument( '-pm', '--problem', type=str,
      nargs='?', default='toymap.json',
      help='what problem to test on' )
    return parser.parse_args()

def main():
    args = get_args()

    with open(args.problem) as f:
        config = json.load(f)
    map = np.load(config['map'])

    if args.method == 'rrt':
        solver = RRT_Solver( map, samples=args.num_samples,
          connect_prob=args.connect_prob, step_length=args.step )
    elif args.method == 'brrt':
        solver = BRRT_Solver( map, samples=args.num_samples,
          connect_prob=args.connect_prob, step_length=args.step )
    elif args.method == 'prm':
        solver = PRM_Solver( map, samples=args.num_samples,
          connect_prob=args.connect_prob, step_length=args.step )
    elif args.method == 'afp':
        solver = AFP_Solver( map, samples=args.num_samples,
          connect_prob=args.connect_prob, step_length=args.step )
    else:
        print('unknown method:', args.method)
        return

    solution = solver.solve(config['start'], config['goal'])
    # print(solution)
    # draw_map(map, config['start'], config['goal'], path=solution, wfil='toymap_sol.png')
    draw_path(map, config['start'], config['goal'], solver, path=solution, wfil='toymap_sol.png')


if __name__ == '__main__':
    main()
