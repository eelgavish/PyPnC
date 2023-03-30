from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.finder.dijkstra import DijkstraFinder
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple
import math
from rdp import rdp
import yaml
from writing_yaml_test import write_yamls

print("Running...")
mapfile = "maps/Travis.png"
heightmap = cv.imread(mapfile,0)

# Scale the matrix linearly to range [0, 100]
min_val = np.min(heightmap)
max_val = np.max(heightmap)
scaled_heightmap = 100 * ((heightmap - min_val) / (max_val - min_val))

blur = cv.blur(scaled_heightmap,(25,25))
print("Blurred...")
grid = Grid(matrix=blur)
#grid = Grid(matrix=scaled_heightmap)

start = grid.node(0,0)
end = grid.node(grid.width-1,grid.height-1)

#finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
finder = DijkstraFinder(diagonal_movement=DiagonalMovement.always)
path, runs = finder.find_path(start, end, grid)
print("Solution Found.")

print('operations:', runs, 'path length:', len(path))
#print(grid.grid_str(path=path, start=start, end=end))
testList2 = [[elem1, elem2] for elem1, elem2 in path]

desiredNumberOfPoints = 10
print(len(testList2))
e = (len(testList2) / (3 * desiredNumberOfPoints)) * 2
testList3 = rdp(testList2, epsilon=e)
print(testList3)

# Write this stuff to a yaml file
write_yamls(testList3)

plt.figure(1)
plt.imshow(scaled_heightmap, cmap='hot')
plt.plot(*zip(*testList3))
plt.figure(2)
plt.imshow(blur, cmap='hot')
plt.plot(*zip(*testList3))
plt.show()
print("Done.")