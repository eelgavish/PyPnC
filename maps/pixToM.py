import cv2 as cv
import numpy as np
import csv
import imutils
import matplotlib.pyplot as plt

def writePNGtoCSV(mapfile, samples, scalexy, scalez):

    """Transforms the input mapfile png to a txt file with full point data in meters
    Input:
        mapfile | png file path
        samples | size of output array
        scalexy | ground scaling factor in [pixels/m]
        scalez  | height scaling factor in [m] (represents full range of height min-max)
    """

    heightmap = cv.imread(mapfile,0)
    blur_image = cv.blur(heightmap, (15,15))

    shape0 = blur_image.shape
    resize_heightmap = imutils.resize(blur_image, width = samples, inter=cv.INTER_LINEAR)
    shape = resize_heightmap.shape

    # Scale the matrix linearly to range [0, scalez]
    min_val = np.min(blur_image)
    max_val = np.max(blur_image)
    scaled_heightmap = scalez * ((resize_heightmap - min_val) / (max_val - min_val))
    coordinates = np.zeros((shape[0]*shape[1],3))

    k = 0
    for i in range(shape[0]):
        for j in range(shape[1]):
            coordinates[k,:] = np.array([j*scalexy/shape[0], i*scalexy/shape[1], scaled_heightmap[i,j]])
            k = k+1

    with open(mapfile+'.txt', 'w') as f:
        writer = csv.writer(f, delimiter=',')
        writer.writerows(coordinates)

    return 0





# if __name__ == "__main__":
#     writePNGtoCSV("maps/SimpleMaze.png", 100, 30, 1)