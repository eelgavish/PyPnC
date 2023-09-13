from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.core.util import smoothen_path
from pathfinding.finder.a_star import AStarFinder
from pathfinding.finder.dijkstra import DijkstraFinder

from slope_finder import SlopeFinder

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple
import math
from rdp import rdp
import yaml
from write_yamls import write_yamls

import csv

def trajectory_optimizer(mapfile, ini, fin, blur_weight=10, dist_weight=1, slope_weight=1, desired_pts=10):
    heightfile = mapfile + ".txt"
    heightfield = np.genfromtxt(heightfile, delimiter=',')
    heightmap = heightfield[:,2].reshape(100,100)
    resolution = heightfield[1,0]
    scaled_heightmap = (heightmap+1)*100

    blur_image = cv.blur(scaled_heightmap,(blur_weight,blur_weight))
    grid = Grid(matrix=blur_image)

    with open('mapincsvform', 'w') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(blur_image)

    start = grid.node(ini[0],ini[1])
    end = grid.node(fin[0],fin[1])

    finder = SlopeFinder(resolution=resolution,slope_weight=slope_weight,diagonal_movement=DiagonalMovement.always)
    path, runs = finder.find_path(start, end, grid)
    print('operations:', runs, 'path length:', len(path))
    print(grid.grid_str(path=path, start=start, end=end))
    testList2 = [[elem1, elem2] for elem1, elem2 in path]

    plt.figure(2)
    plt.imshow(scaled_heightmap, cmap='hot')
    plt.plot(*zip(*testList2))
    plt.gca().invert_yaxis()
    plt.show()

    e = (len(testList2) / (3 * desired_pts)) * 2
    testList3 = rdp(testList2, epsilon=e)

    # Write this stuff to a yaml file
    yamlList = [[point[0]*resolution, point[1]*resolution, heightmap[point[1], point[0]]] for point in testList3]

    # Check the lengths of links and split them if they are too long
    yamlList2 = [yamlList[0]]
    for i in range(len(yamlList)-1):
        pt1 = yamlList[i]
        pt2 = yamlList[i+1]
        maxLength = 2
        numToSplit = math.floor(math.dist(pt1,pt2)/maxLength) # Splits into segments no longer than maxLength
        if numToSplit > 0:
            x1 = pt1[0]
            y1 = pt1[1]
            z1 = pt1[2]
            x2 = pt2[0]
            y2 = pt2[1]
            z2 = pt2[2]
            xdist = (x2-x1)/(numToSplit+1)
            ydist = (y2-y1)/(numToSplit+1)
            zdist = (z2-z1)/(numToSplit+1)
            for j in range(numToSplit):
                k = j+1
                add = np.array([[x1+k*xdist, y1+k*ydist, z1+k*zdist]])
                yamlList2 = np.append(yamlList2, add)
        yamlList2 = np.append(yamlList2, yamlList[i+1])

    yamlList2 = yamlList2.reshape(int(len(yamlList2)/3),3)

    #print(yamlList2)
    write_yamls(yamlList2)

    plt.figure(1)
    plt.imshow(scaled_heightmap, cmap='hot')
    plt.plot(*zip(*testList3))
    plt.gca().invert_yaxis()
    plt.show()

    print("Done.")

    return 0

# if __name__ == "__main__":
#     print("Running...")
#     mapfile = "maps/Travis.png"

#     heightfile = mapfile + ".txt"
#     heightfield = np.genfromtxt(heightfile, delimiter=',')
#     heightmap = heightfield[:,2].reshape(100,100)
#     resolution = heightfield[1,0]
#     scaled_heightmap = heightmap*100

#     blur = cv.blur(scaled_heightmap,(25,25))
#     print("Blurred...")
#     grid = Grid(matrix=blur)

    # plt.figure(1)
    # plt.imshow(scaled_heightmap, cmap='hot')
    # plt.figure(2)
    # plt.imshow(blur, cmap='hot')
    # plt.show()
    # print("Done.")

    # start = grid.node(0,0)
    #end = grid.node(grid.width-1,grid.height-1)
    # end = grid.node(80,80)

    # finder = SlopeFinder(resolution=resolution,dist_weight=1,slope_weight=1,diagonal_movement=DiagonalMovement.always)

    # path, runs = finder.find_path(start, end, grid)

    # testList2 = [[elem1, elem2] for elem1, elem2 in path]

    # desiredNumberOfPoints = 20
    # e = (len(testList2) / (3 * desiredNumberOfPoints)) * 2
    # testList3 = rdp(testList2, epsilon=e)

    # # Write this stuff to a yaml file
    # yamlList = [[point[0]*resolution, point[1]*resolution, heightmap[point[0], point[1]]] for point in testList3]
    # print(yamlList)

    # # Check the lengths of links and split them if they are too long
    # yamlList2 = [yamlList[0]]
    # for i in range(len(yamlList)-1):
    #     pt1 = yamlList[i]
    #     pt2 = yamlList[i+1]
    #     numToSplit = math.floor(math.dist(pt1,pt2)/2)
    #     if numToSplit > 0:
    #         x1 = pt1[0]
    #         y1 = pt1[1]
    #         z1 = pt1[2]
    #         x2 = pt2[0]
    #         y2 = pt2[1]
    #         z2 = pt2[2]
    #         xdist = (x2-x1)/(numToSplit+1)
    #         ydist = (y2-y1)/(numToSplit+1)
    #         zdist = (z2-z1)/(numToSplit+1)
    #         for j in range(numToSplit):
    #             k = j+1
    #             add = np.array([[x1+k*xdist, y1+k*ydist, z1+k*zdist]])
    #             yamlList2 = np.append(yamlList2, add)
    #     yamlList2 = np.append(yamlList2, yamlList[i+1])

    # yamlList2 = yamlList2.reshape(int(len(yamlList2)/3),3)

    # write_yamls(yamlList2)

    # plt.figure(1)
    # plt.imshow(scaled_heightmap, cmap='hot')
    # plt.plot(*zip(*testList2))
    # plt.figure(2)
    # plt.imshow(blur, cmap='hot')
    # plt.plot(*zip(*testList2))
    # plt.show()
    # print("Done.")