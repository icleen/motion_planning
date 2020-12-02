
from policy_iteration import policy_iteration
from graph_map import GridMap
from mdp_map import MDPMap


def run_3(map, outpath='outputs/'):
    values, policy, path, iters = policy_iteration(map)
    map.display_values(values, outpath + 'values.png', iters=iters)
    map.display_policy(policy, outpath + 'policy.png')
    map.display_map(path, [], outpath + 'actions{}.png'.format(len(path)))


map0 = MDPMap('map0.txt', MDPMap._ACTIONS1)
map1 = MDPMap('map1.txt', MDPMap._ACTIONS1)
map2 = MDPMap('map2.txt', MDPMap._ACTIONS3)

print('test 3.1')
run_3(map0, 'outputs3/mdp1_map0_iu_')
run_3(map1, 'outputs3/mdp1_map1_iu_')

print('test 3.2')
run_3(map0, 'outputs3/mdp2_map0_id_')
run_3(map1, 'outputs3/mdp2_map1_id_')
