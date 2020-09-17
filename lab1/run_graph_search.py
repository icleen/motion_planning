import graph_search
import os, sys, argparse

def run_dfs(graph,actions=graph_search._ACTIONS):
    return graph_search.dfs(
      graph.init_pos, graph.transition, graph.is_goal, actions )
def run_bfs(graph,actions=graph_search._ACTIONS):
    return graph_search.bfs(
      graph.init_pos, graph.transition, graph.is_goal, actions )
def run_ucs(graph,actions=graph_search._ACTIONS):
    return graph_search.uniform_cost_search(
      graph.init_pos, graph.transition, graph.is_goal, actions )
def run_astar(graph, actions=graph_search._ACTIONS, heuristic='euc'):
    heur = graph.euclidean_heuristic if heuristic=='euc' else g.manhatten_heuristic
    return graph_search.a_star_search(
      graph.init_pos, graph.transition, graph.is_goal, actions, heur )


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
    graph = graph_search.GridMap(config.map)
    if config.algorithm == 'dfs':
        result = run_dfs(graph)
    elif config.algorithm == 'idfs':
        result = run_idfs(graph)
    elif config.algorithm == 'bfs':
        result = run_bfs(graph)
    elif config.algorithm == 'ucs':
        result = run_ucs(graph)
    elif config.algorithm == 'ucs2':
        result = run_ucs(graph, graph_search._ACTIONS_2)
    elif config.algorithm == 'astar':
        result = run_astar(graph)
    elif config.algorithm == 'astar2':
        result = run_astar(graph, heuristic='man')
    elif config.algorithm == 'astar3':
        result = run_astar(graph, actions=graph_search._ACTIONS_2)
    elif config.algorithm == 'graph':
        result = run_graph(graph)
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
