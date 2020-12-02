
import os

from discrete_planners import bfs
from graph_map import GridMap
from mdp_map import MDPMap


def run_1(map, num=0, outpath='outputs/'):
    transition = lambda state, action : map.transition(state, action)[0][1]
    for test in range(5):
        map.simode = False
        result = bfs(map.init_pos, transition, map.is_goal, list(map.actions.keys()))
        # (path, action_path), visited = result
        action_path = result[0][1]

        state = map.init_pos
        path = [state]
        for act in action_path:
            state = map.transition(state, act, simode=True)
            path.append( state )
        map.display_map(path, [], outpath + 'map{}_test{}.png'.format(num, test))

os.makedirs('outputs1/', exist_ok=True)

map0 = MDPMap('map0.txt', MDPMap._ACTIONS1)
map1 = MDPMap('map1.txt', MDPMap._ACTIONS1)
map2 = MDPMap('map2.txt', MDPMap._ACTIONS3)

print('test 1.1')
run_1(map0, 0, 'outputs1/mdp1_')
run_1(map1, 1, 'outputs1/mdp1_')

print('test 1.2')
map0.actions = MDPMap._ACTIONS2
map1.actions = MDPMap._ACTIONS2
run_1(map0, 0, 'outputs1/mdp2_')
run_1(map1, 1, 'outputs1/mdp2_')

print('test 1.3')
map0.actions = MDPMap._ACTIONS3
map1.actions = MDPMap._ACTIONS3
run_1(map0, 0, 'outputs1/mdp3_')
run_1(map1, 1, 'outputs1/mdp3_')
run_1(map2, 2, 'outputs1/mdp3_')

print('test 1.4')
map0.actions = MDPMap._ACTIONS4
map1.actions = MDPMap._ACTIONS4
map2.actions = MDPMap._ACTIONS4
run_1(map0, 0, 'outputs1/mdp4_')
run_1(map1, 1, 'outputs1/mdp4_')
run_1(map2, 2, 'outputs1/mdp4_')
