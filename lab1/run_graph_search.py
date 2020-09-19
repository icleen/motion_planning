import graph_search
import os, sys, argparse
import os.path as osp

def run_dfs(graph,actions=graph_search._ACTIONS):
    return graph_search.dfs(
      graph.init_pos, graph.transition, graph.is_goal, actions )
def run_idfs(graph,actions=graph_search._ACTIONS):
    return graph_search.idfs(
      graph.init_pos, graph.transition, graph.is_goal, actions,
      maxdepth=graph.rows*graph.cols )
def run_bfs(graph,actions=graph_search._ACTIONS):
    return graph_search.bfs(
      graph.init_pos, graph.transition, graph.is_goal, actions )
def run_ucs(graph,actions=graph_search._ACTIONS):
    return graph_search.uniform_cost_search(
      graph.init_pos, graph.transition, graph.is_goal, actions )
def run_astar(graph, actions=graph_search._ACTIONS, heuristic='euc'):
    heur = graph.euclidean_heuristic if heuristic=='euc' else graph.manhattan_heuristic
    return graph_search.a_star_search(
      graph.init_pos, graph.transition, graph.is_goal, actions, heur )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
      "-a", "--algorithm", type=str, default='dfs',
      help="decide which algorithm to run\n options: [dfs, dfs2, idfs, bfs, ucs, ucs2, astar, astar2, astar3, astar4, graph]"
    )
    parser.add_argument(
      "-m", "--map", type=str, default='map1.txt',
      help="path to output directory"
    )
    parser.add_argument(
      "-p", "--pose", nargs='?', default=False, const=True,
      help="whether to use pose actions"
    )
    config = parser.parse_args()
    graph = graph_search.GridMap(config.map)
    actions = graph_search._ACTIONS.copy()
    if config.pose:
        actions = graph_search._ACTIONS_3
    if config.algorithm == 'dfs':
        result = run_dfs(graph, actions=actions)
    elif config.algorithm == 'dfs2':
        actions.reverse()
        result = run_dfs(graph, actions=actions)
    elif config.algorithm == 'idfs':
        result = run_idfs(graph, actions=actions)
    elif config.algorithm == 'bfs':
        result = run_bfs(graph, actions=actions)
    elif config.algorithm == 'ucs':
        result = run_ucs(graph, actions=actions)
    elif config.algorithm == 'ucs2':
        result = run_ucs(graph, graph_search._ACTIONS_2)
    elif config.algorithm == 'astar':
        result = run_astar(graph, actions=actions)
    elif config.algorithm == 'astar2':
        result = run_astar(graph, actions=actions, heuristic='man')
    elif config.algorithm == 'astar3':
        result = run_astar(graph, actions=graph_search._ACTIONS_2)
    elif config.algorithm == 'astar4':
        result = run_astar(graph, actions=graph_search._ACTIONS_2, heuristic='man')
    elif config.algorithm == 'graph':
        result = run_graph(graph)
    else:
        print('Unknown algorithm:', config.algorithm)
        return 1

    writefile = 'result_{}_{}'.format(config.algorithm, config.map)
    if config.pose:
        writefile = writefile.replace('.txt', '_pose.txt')
    imagepath = osp.join('images',writefile.replace('.txt', '.png'))
    path, visited = result
    path, action_path = path
    with open(osp.join('outputs',writefile), 'w') as f:
        f.write('Path:\n')
        f.write(str(path))
        f.write('\n')
        f.write('Actions:\n')
        f.write(str(action_path))
        f.write('\n')
        f.write('Visited:\n')
        f.write(str(visited))
        f.write('\n')
        f.write('pic: {}\n'.format(imagepath))
    graph.display_map(path,visited,imagepath)


if __name__ == '__main__':
    main()
