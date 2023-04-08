from pathfinding.finder.a_star import AStarFinder, MAX_RUNS, TIME_LIMIT
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.heuristic import euclidean, null
from pathfinding.core.util import SQRT2


class SlopeFinder(AStarFinder):

    def __init__(self, weight=1,
                 diagonal_movement=DiagonalMovement.never,
                 time_limit=TIME_LIMIT,
                 max_runs=MAX_RUNS,
                 resolution=1.0,
                 dist_weight = 1.0,
                 slope_weight = 1.0):
        super(SlopeFinder, self).__init__(
            heuristic=null,
            weight=weight,
            diagonal_movement=diagonal_movement,
            time_limit=time_limit,
            max_runs=max_runs)
        self.resolution = resolution
        self.dist_weight = dist_weight
        self.slope_weight = slope_weight
        
    def calc_cost(self, node_a, node_b):
        """
        get the distance between current node and the neighbor (cost)
        """
        if node_b.x - node_a.x == 0 or node_b.y - node_a.y == 0:
            # direct neighbor - distance is 1
            ng = self.resolution
        else:
            # not a direct neighbor - diagonal movement
            ng = SQRT2 * self.resolution

        # weight for weighted algorithms
        if self.weighted:
            ng *= self.dist_weight
            ng += ((node_b.weight - node_a.weight)/self.resolution) * self.slope_weight

        return node_a.g + ng