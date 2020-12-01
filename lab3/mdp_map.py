
from graph_map import *

class MDPMap(GridMap):
    """docstring for MDPMap."""

    _ACTIONS1 = {
      'u':[(0.8, 'u'), (0.1, 'l'), (0.1, 'r')],
      'd':[(0.8, 'd'), (0.1, 'l'), (0.1, 'r')],
      'l':[(0.8, 'l'), (0.1, 'u'), (0.1, 'd')],
      'r':[(0.8, 'r'), (0.1, 'u'), (0.1, 'd')],
    }

    _ACTIONS2 = {
      'u':[(0.6, 'u'), (0.2, 'l'), (0.2, 'r')],
      'd':[(0.6, 'd'), (0.2, 'l'), (0.2, 'r')],
      'l':[(0.6, 'l'), (0.2, 'u'), (0.2, 'd')],
      'r':[(0.6, 'r'), (0.2, 'u'), (0.2, 'd')],
    }

    _ACTIONS3 = {
      'u':[(0.8, 'u'), (0.1, 'ne'), (0.1, 'nw')],
      'd':[(0.8, 'd'), (0.1, 'se'), (0.1, 'sw')],
      'l':[(0.8, 'l'), (0.1, 'sw'), (0.1, 'nw')],
      'r':[(0.8, 'r'), (0.1, 'se'), (0.1, 'ne')],
      'nw':[(0.8, 'nw'), (0.1, 'u'), (0.1, 'l')],
      'ne':[(0.8, 'ne'), (0.1, 'u'), (0.1, 'r')],
      'sw':[(0.8, 'sw'), (0.1, 'd'), (0.1, 'l')],
      'se':[(0.8, 'se'), (0.1, 'd'), (0.1, 'r')],
    }

    _ACTIONS4 = {
      'u':[(0.7, 'u'), (0.1, 'ne'), (0.1, 'nw'), (0.05, 'l'), (0.05, 'r')],
      'd':[(0.7, 'd'), (0.1, 'se'), (0.1, 'sw'), (0.05, 'l'), (0.05, 'r')],
      'l':[(0.7, 'l'), (0.1, 'sw'), (0.1, 'nw'), (0.05, 'u'), (0.05, 'd')],
      'r':[(0.7, 'r'), (0.1, 'se'), (0.1, 'ne'), (0.05, 'u'), (0.05, 'd')],
      'nw':[(0.7, 'nw'), (0.1, 'u'), (0.1, 'l'), (0.05, 'ne'), (0.05, 'sw')],
      'ne':[(0.7, 'ne'), (0.1, 'u'), (0.1, 'r'), (0.05, 'nw'), (0.05, 'se')],
      'sw':[(0.7, 'sw'), (0.1, 'd'), (0.1, 'l'), (0.05, 'se'), (0.05, 'nw')],
      'se':[(0.7, 'se'), (0.1, 'd'), (0.1, 'r'), (0.05, 'sw'), (0.05, 'ne')],
    }

    def __init__(self, map_path=None, actions=None, simode=False, goalval=10, stateval=0):
        super(MDPMap, self).__init__(map_path)
        self.actions = actions
        if actions is None:
            self.actions = _ACTIONS1
        self.simode = simode
        self.goalval = goalval
        self.stateval = stateval

    def transition(self, state, action):
        outstates = []
        for prob, act in self.actions[action]:
            outstates.append( (prob, super().transition(state, act)) )

        if self.simode:
            probs = [s[0] for s in outstates]
            outstates = [s[1] for s in outstates]
            idx = np.random.choice(np.arange(len(outstates)), p=probs)
            return outstates[idx]
        return outstates

    def value(self, state, action, nstate):
        if self.is_goal(nstate):
            return self.goalval
        return self.stateval

    def get_actions(self):
        return list(self.actions.keys())


def main():
    map = MDPMap('map1.txt')
    states = map.transition(map.init_pos, 'u')
    print('init:', map.init_pos)
    print('states:', states)

    map.simode = True
    print('samples:')
    for _ in range(5):
        state = map.transition(map.init_pos, 'u')
        print('state:', state)

if __name__ == '__main__':
    main()
