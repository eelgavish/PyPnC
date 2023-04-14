from pixToM import writePNGtoCSV
from trajectory_optimization import trajectory_optimizer

def setup(mapfile, ini, fin, scalexy, scalez, dist_weight=1, slope_weight=1):

    writePNGtoCSV(mapfile, 100, scalexy, scalez) # Converts height map from .png to .txt file

    trajectory_optimizer(mapfile, ini, fin, dist_weight=dist_weight, slope_weight=slope_weight, desired_pts=20) # 

    return 0

if __name__ == "__main__":
    setup("maps/RollingHillsHeight.png", (25,5), (10,80), 10, 1, dist_weight=4, slope_weight=1)