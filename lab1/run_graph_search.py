import graph_search
import os, sys, argparse

def run_dfs(map_path,actions=graph_search._ACTIONS):
    g = graph_search.GridMap(map_path)
    res = graph_search.dfs(g.init_pos, g.transition, g.is_goal, actions)
    g.display_map(res[0][0],res[1])
def run_bfs(map_path,actions=graph_search._ACTIONS):
    g = graph_search.GridMap(map_path)
    res = graph_search.bfs(g.init_pos, g.transition, g.is_goal, actions)
    g.display_map(res[0][0],res[1])
def run_ucs(map_path,actions=graph_search._ACTIONS):
    g = graph_search.GridMap(map_path)
    res = graph_search.uniform_cost_search(g.init_pos, g.transition, g.is_goal, actions)
    g.display_map(res[0][0],res[1])
def run_astar(map_path, actions=graph_search._ACTIONS, heuristic='euc'):
    g = graph_search.GridMap(map_path)
    heur = g.euclidean_heuristic if heuristic=='euc' else g.manhatten_heuristic
    res = graph_search.a_star_search(g.init_pos, g.transition, g.is_goal, actions,heur)
    g.display_map(res[0][0],res[1])


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
    if config.algorithm == 'dfs':
        run_dfs(config.map)
    elif config.algorithm == 'idfs':
        run_idfs(config.map)
    elif config.algorithm == 'bfs':
        run_bfs(config.map)
    elif config.algorithm == 'ucs':
        run_ucs(config.map)
    elif config.algorithm == 'ucs2':
        run_ucs(config.map, graph_search._ACTIONS_2)
    elif config.algorithm == 'astar':
        run_astar(config.map)
    elif config.algorithm == 'astar2':
        run_astar(config.map, heuristic='man')
    elif config.algorithm == 'astar3':
        run_astar(config.map, actions=graph_search._ACTIONS_2)
    elif config.algorithm == 'graph':
        run_graph(config.map)
    else:
        print('Unknown algorithm:', config.algorithm)
        return 1


if __name__ == '__main__':
    main()
