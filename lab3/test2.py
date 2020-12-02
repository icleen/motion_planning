
from value_iteration import value_iter
from graph_map import GridMap
from mdp_map import MDPMap


def run_2(map, outpath='outputs/'):
    values, policy, path, iters = value_iter(map)
    map.display_values(values, outpath + 'values.png', iters=iters)
    map.display_policy(policy, outpath + 'policy.png')
    map.display_map(path, [], outpath + 'actions{}.png'.format(len(path)))


map0 = MDPMap('map0.txt', MDPMap._ACTIONS1)
map1 = MDPMap('map1.txt', MDPMap._ACTIONS1)
map2 = MDPMap('map2.txt', MDPMap._ACTIONS3)

print('test 2.1')
run_2(map0, 'outputs2/mdp1_map0_l08_')
run_2(map1, 'outputs2/mdp1_map1_l08_')

print('test 2.2')
map0.discount = 0.9
map1.discount = 0.9
run_2(map0, 'outputs2/mdp2_map0_l09_')
run_2(map1, 'outputs2/mdp2_map1_l09_')
map0.discount = 0.5
map1.discount = 0.5
run_2(map0, 'outputs2/mdp2_map0_l05_')
run_2(map1, 'outputs2/mdp2_map1_l05_')
map0.discount = 0.2
map1.discount = 0.2
run_2(map0, 'outputs2/mdp2_map0_l02_')
run_2(map1, 'outputs2/mdp2_map1_l02_')
map0.discount = 0.0
map1.discount = 0.0
run_2(map0, 'outputs2/mdp2_map0_l00_')
run_2(map1, 'outputs2/mdp2_map1_l00_')

print('test 2.3')
map0.actions = MDPMap._ACTIONS3
map1.actions = MDPMap._ACTIONS3
map0.discount = 0.8
map1.discount = 0.8
map2.discount = 0.8
run_2(map0, 'outputs2/mdp3_map0_l08_')
run_2(map1, 'outputs2/mdp3_map1_l08_')
run_2(map2, 'outputs2/mdp3_map2_l08_')

print('test 2.4')
map0.actions = MDPMap._ACTIONS4
map1.actions = MDPMap._ACTIONS4
map2.actions = MDPMap._ACTIONS4
run_2(map0, 'outputs2/mdp4_map0_l08_')
run_2(map1, 'outputs2/mdp4_map1_l08_')
run_2(map2, 'outputs2/mdp4_map2_l08_')

print('test 2.5')
map0.actions = MDPMap._ACTIONS1
map1.actions = MDPMap._ACTIONS1
map0.goalval = 1.0
map1.goalval = 1.0
run_2(map0, 'outputs2/mdp5_map0_gv1_')
run_2(map1, 'outputs2/mdp5_map1_gv1_')
map0.goalval = 10.0
map0.stateval = -1.0
map1.goalval = 10.0
map1.stateval = -1.0
run_2(map0, 'outputs2/mdp5_map0_gv10-1_')
run_2(map1, 'outputs2/mdp5_map1_gv10-1_')
map0.goalval = 10.0
map0.stateval = 0.0
map0.cornerval = -5.0
map1.goalval = 10.0
map1.stateval = 0.0
map1.cornerval = -5.0
run_2(map0, 'outputs2/mdp5_map0_gv10-5_')
run_2(map1, 'outputs2/mdp5_map1_gv10-5_')
