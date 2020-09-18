You can run any of the tests with command line arguments.
'python run_graph_search.py -a [algorithm] -m [map_file] -h (help)'
The options for algorithm are:
dfs (depth first search), idfs (iterative dfs), bfs (breadth first search),
ucs (uniform cost search), ucs2 (ucs with diagonal actions),
astar (A* search with euclidean heuristic), astar2 (A* with manhatten heuristic),
astar3 (A* with euclidean heuristic and diagonal actions),

The default map_file is ./map1.txt but you can replace it with any file path.  
