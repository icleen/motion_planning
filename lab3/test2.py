
from value_iteration import value_iter
from graph_map import GridMap
from mdp_map import MDPMap


def run_2(map, num=0, outpath='outputs/'):

    values = value_iter(map)
    import pdb; pdb.set_trace()


map0 = MDPMap('map0.txt', MDPMap._ACTIONS1)
map1 = MDPMap('map1.txt', MDPMap._ACTIONS1)
map2 = MDPMap('map2.txt', MDPMap._ACTIONS3)

print('test 2.1')
run_2(map0, 0, 'outputs2/mdp1_')
# run_2(map1, 1, 'outputs2/mdp1_')
#
# print('test 2.2')
# map0.actions = MDPMap._ACTIONS2
# map1.actions = MDPMap._ACTIONS2
# run_2(map0, 0, 'outputs2/mdp2_')
# run_2(map1, 1, 'outputs2/mdp2_')
#
# print('test 2.3')
# map0.actions = MDPMap._ACTIONS3
# map1.actions = MDPMap._ACTIONS3
# run_2(map0, 0, 'outputs2/mdp3_')
# run_2(map1, 1, 'outputs2/mdp3_')
# run_2(map2, 2, 'outputs2/mdp3_')
#
# print('test 2.4')
# map0.actions = MDPMap._ACTIONS4
# map1.actions = MDPMap._ACTIONS4
# map2.actions = MDPMap._ACTIONS4
# run_2(map0, 0, 'outputs2/mdp4_')
# run_2(map1, 1, 'outputs2/mdp4_')
# run_2(map2, 2, 'outputs2/mdp4_')
