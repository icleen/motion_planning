
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

    # _GOAL_COLOR = 1.5
    # _PATH_COLOR = 0.25
    # _INIT_COLOR = 0.0

    def __init__(self, map_path=None, actions=None, goalval=10, stateval=0):
        super(MDPMap, self).__init__(map_path)
        self.actions = actions
        if actions is None:
            self.actions = _ACTIONS1
        self.goalval = goalval
        self.stateval = stateval

    def transition(self, state, action, simode=False):
        outstates = []
        for prob, act in self.actions[action]:
            outstates.append( (prob, super().transition(state, act)) )

        if simode:
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

    def display_values(self, values={}, filename=None, plot_show=False, iters=None):
        '''
        Visualize the map read in. Optionally display the resulting plan and visisted nodes

        path - a list of tuples describing the path take from init to goal
        visited - a set of tuples describing the states visited during a search
        filename - relative path to file where image will be saved
        '''
        plotter.clf()
        display_grid = np.array(self.occupancy_grid, dtype=np.float32)+1.0

        # Color all visited nodes if requested
        for state in values:
            val = values[state]
            disp_col = self._INIT_COLOR + self._PATH_COLOR_RANGE*(val)/self.goalval
            display_grid[state[0],state[1]] = disp_col
        display_grid[self.goal] = self._GOAL_COLOR

        if iters is not None:
            plotter.title('iterations: {}'.format(iters))

        # Plot display grid for visualization
        imgplot = plotter.imshow(display_grid)
        # Set interpolation to nearest to create sharp boundaries
        imgplot.set_interpolation('nearest')
        # Set color map to diverging style for contrast
        imgplot.set_cmap('Spectral')
        if filename is not None:
            plotter.savefig(filename)
        if plot_show:
            plotter.show()

    def display_policy(self, policy={}, filename=None, plot_show=False):
        '''
        Visualize the map read in. Optionally display the resulting plan and visisted nodes

        path - a list of tuples describing the path take from init to goal
        visited - a set of tuples describing the states visited during a search
        filename - relative path to file where image will be saved
        '''
        plotter.clf()
        display_grid = np.array(self.occupancy_grid, dtype=np.float32)*self._WALL_COLOR

        actvals = {}
        for ai, act in enumerate(self.actions):
            aval = self._INIT_COLOR + self._PATH_COLOR_RANGE*(ai+1)/len(self.actions)
            actvals[act] = aval

        # Color all visited nodes if requested
        for state in policy:
            action = policy[state]
            display_grid[state[0],state[1]] = actvals[action]
        display_grid[self.goal] = self._GOAL_COLOR
        # print('grid:', display_grid)

        # Plot display grid for visualization
        imgplot = plotter.imshow(display_grid)
        # Set interpolation to nearest to create sharp boundaries
        imgplot.set_interpolation('nearest')
        # Set color map to diverging style for contrast
        imgplot.set_cmap('Spectral')
        if filename is not None:
            plotter.savefig(filename)
        if plot_show:
            plotter.show()


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
