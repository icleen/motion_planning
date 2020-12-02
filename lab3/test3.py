
import os
import random
from collections import defaultdict

from policy_iteration import policy_iteration
from graph_map import GridMap
from mdp_map import MDPMap


def run_3(map, outpath='outputs/', init_policy=None):
    values, policy, path, iters = policy_iteration(map, policy=init_policy)
    map.display_values(values, outpath + 'values.png', iters=iters)
    map.display_policy(policy, outpath + 'policy.png')
    map.display_map(path, [], outpath + 'actions{}.png'.format(len(path)))

os.makedirs('outputs3/', exist_ok=True)

map0 = MDPMap('map0.txt', MDPMap._ACTIONS1, discount=0.8)
map1 = MDPMap('map1.txt', MDPMap._ACTIONS1, discount=0.8)

print('test 3.1')
policy = defaultdict(lambda: 'u')
run_3(map0, 'outputs3/mdp1_map0_iu_', policy)
run_3(map1, 'outputs3/mdp1_map1_iu_', policy)

print('test 3.2')
policy = defaultdict(lambda: 'd')
run_3(map0, 'outputs3/mdp2_map0_id_', policy)
run_3(map1, 'outputs3/mdp2_map1_id_', policy)

print('test 3.3')
for test in range(1, 6):
    policy = defaultdict(lambda: random.choice(list(MDPMap._ACTIONS1.keys())))
    run_3(map0, 'outputs3/mdp3_map0_ir{}_'.format(test), policy)
    policy = defaultdict(lambda: random.choice(list(MDPMap._ACTIONS1.keys())))
    run_3(map1, 'outputs3/mdp3_map1_ir{}_'.format(test), policy)
