import cv2 as cv
import numpy as np
import csv
import imutils

def writePNGtoCSV(mapfile, samples, scalexy, scalez):
    """Transforms the input mapfile png to a txt file with full point data in meters
    Input:
        mapfile | png file path
        samples | size of output array
        scalexy | ground scaling factor in [pixels/m]
        scalez  | height scaling factor in [m] (represents full range of height min-max)
    """

    heightmap = cv.imread(mapfile,0)
    shape0 = heightmap.shape
    heightmap = imutils.resize(heightmap, width = samples, inter=cv.INTER_LINEAR)
    shape = heightmap.shape
    # Scale the matrix linearly to range [0, scalez]
    min_val = np.min(heightmap)
    max_val = np.max(heightmap)
    scaled_heightmap = scalez * ((heightmap - min_val) / (max_val - min_val))

    coordinates = np.zeros((shape[0]*shape[1],3))

    k = 0
    for i in range(shape[0]):
        for j in range(shape[1]):
            coordinates[k,:] = np.array([i/scalexy*shape0[0]/shape[0], j/scalexy*shape0[1]/shape[1], scaled_heightmap[i,j]])
            k = k+1

    with open('out.txt', 'w') as f:
        writer = csv.writer(f, delimiter=',')
        writer.writerows(coordinates)

writePNGtoCSV("maps/Travis.png", 100, 10, 5)