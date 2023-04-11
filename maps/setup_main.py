from pixToM import writePNGtoCSV
from trajectory_optimization import trajectory_optimizer

def setup(mapfile, ini, fin, scalexy, scalez, dist_weight=1, slope_weight=1):
    writePNGtoCSV(mapfile, 100, scalexy, scalez)
    trajectory_optimizer(mapfile, ini, fin, dist_weight=dist_weight, slope_weight=slope_weight, desired_pts=50)
    return 0

if __name__ == "__main__":
    setup("maps/RollingHillsHeight.png", (13,60), (80,70), 10, 1, slope_weight=10)