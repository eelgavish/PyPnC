from pixToM import writePNGtoCSV
from trajectory_optimization import trajectory_optimizer

def setup(mapfile, ini, fin, scalexy, scalez, slope_weight=0.5):

    writePNGtoCSV(mapfile, 100, scalexy, scalez) # Converts height map from .png to .txt file

    trajectory_optimizer(mapfile, ini, fin, slope_weight=slope_weight, desired_pts=50) # 

    return 0

if __name__ == "__main__":
    setup("maps/SimpleMaze.png", (28,20), (26,92), 10, 1, slope_weight=0.8)