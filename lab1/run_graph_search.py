import graph_search
import os, sys, argparse

def run_dfs(g,actions=graph_search._ACTIONS):
    return graph_search.dfs(g.init_pos, g.transition, g.is_goal, actions)
def run_bfs(g,actions=graph_search._ACTIONS):
    return graph_search.bfs(g.init_pos, g.transition, g.is_goal, actions)
def run_ucs(g,actions=graph_search._ACTIONS):
    return graph_search.uniform_cost_search(g.init_pos, g.transition, g.is_goal, actions)
def run_astar(g, actions=graph_search._ACTIONS, heuristic='euc'):
    heur = g.euclidean_heuristic if heuristic=='euc' else g.manhatten_heuristic
    return graph_search.a_star_search(g.init_pos, g.transition, g.is_goal, actions, heur)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
      "-a", "--algorithm", type=str, default='dfs',
      help="decide which algorithm to run\n options: [dfs, idfs, bfs, ucs, ucs2, astar, graph]"
    )
    parser.add_argument(
      "-m", "--map", type=str, default='map1.txt',
      help="path to output directory"
    )
    config = parser.parse_args()
    g = graph_search.GridMap(config.map)
    if config.algorithm == 'dfs':
        result = run_dfs(g)
    elif config.algorithm == 'idfs':
        result = run_idfs(g)
    elif config.algorithm == 'bfs':
        result = run_bfs(g)
    elif config.algorithm == 'ucs':
        result = run_ucs(g)
    elif config.algorithm == 'ucs2':
        result = run_ucs(g, graph_search._ACTIONS_2)
    elif config.algorithm == 'astar':
        result = run_astar(g)
    elif config.algorithm == 'astar2':
        result = run_astar(g, heuristic='man')
    elif config.algorithm == 'astar3':
        result = run_astar(g, actions=graph_search._ACTIONS_2)
    elif config.algorithm == 'graph':
        result = run_graph(g)
    else:
        print('Unknown algorithm:', config.algorithm)
        return 1

    writefile = 'result_{}_{}.txt'.format(config.algorithm, config.map)
    path, visited = result
    path, action_path = path
    with open(writefile, 'w') as f:
        f.write(str(path))
        f.write('\n')
        f.write(str(action_path))
        f.write('\n')
        f.write(str(visited))
        f.write('\n')
    g.display_map(path,visited)


if __name__ == '__main__':
    main()
