You can run any of the tests with command line arguments.
'python run_graph_search.py -a [algorithm] -m [map_file] -p (2d pose option)'
The options for algorithm are:
dfs (depth first search), dfs2 (dfs with reverse actions), idfs (iterative dfs), bfs (breadth first search),
ucs (uniform cost search), ucs2 (ucs with diagonal actions),
astar (A* search with Euclidean heuristic), astar2 (A* with Manhatten heuristic),
astar3 (A* with euclidean heuristic and diagonal actions),
astar4 (A* with Manhattan heuristic and diagonal actions)

The default map_file is ./map1.txt but you can replace it with any file path.

For instance, to run astar with pose options and Euclidean heuristic, you would run:
'python run_graph_search.py -a astar -m map1.txt -p'

or for dfs with reverse actions on map2:
'python run_graph_search.py -a dfs -m map2.txt'
