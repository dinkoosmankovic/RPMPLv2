from log_parser import LogParser
from planar_2dof_vis import visualize
import yaml
from math import pi


if __name__ == "__main__":
    parser = LogParser("/tmp/plannerData.log")
    path = parser.get_path()
    #for p in path:
    #    print(p)
    #print(path)
    #visualize(path[0], "test.png")
    with open('../data/planar_2dof/scenario2.yaml', 'r') as file:
        obstacles = yaml.safe_load(file)
    visualize(path, obstacles=obstacles, image_file="/home/nermin/RPMPLv2/data/planar_2dof/planar_2dof.gif", is_trajectory=True, fps=10.0) 
    # path = [-2, -2.5]
    # visualize(path, obstacles=obstacles, image_file=None, is_trajectory=False, fps=10.0)
    
