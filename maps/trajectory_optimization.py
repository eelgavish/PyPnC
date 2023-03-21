from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import cv2 as cv
import matplotlib.pyplot as plt
from typing import List, Tuple
import math
from rdp import rdp

print("Running...")
mapfile = "maps/kierran-want-height-map.jpg"
heightmap = cv.imread(mapfile,0)
blur = cv.blur(heightmap,(100,100))

grid = Grid(matrix=blur)

start = grid.node(0,0)
end = grid.node(grid.width-1,grid.height-1)

finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
path, runs = finder.find_path(start, end, grid)
print("Solution Found.")

print('operations:', runs, 'path length:', len(path))
#print(grid.grid_str(path=path, start=start, end=end))
testList2 = [[elem1, elem2] for elem1, elem2 in path]

desiredNumberOfPoints = 10
print(len(testList2))
e = (len(testList2) / (3 * desiredNumberOfPoints)) * 2
print(e)
testList3 = rdp(testList2, epsilon=e)
print(len(testList3))
plt.figure(1)
plt.imshow(heightmap, cmap='hot')
plt.plot(*zip(*testList2))
plt.figure(2)
plt.imshow(blur, cmap='hot')
plt.plot(*zip(*testList3))
plt.show()
print("Done.")