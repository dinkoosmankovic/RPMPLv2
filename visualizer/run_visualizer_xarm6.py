from log_parser import LogParser
from xarm6_vis import visualize
import yaml


if __name__ == "__main__":
    parser = LogParser("/tmp/plannerData.log")
    path = parser.get_path()
    #for p in path:
    #    print(p)
    #print(path)
    #visualize(path[0], "test.png")
    with open('../data/xarm6/scenario1.yaml', 'r') as file:
        obstacles = yaml.safe_load(file)
    visualize(path, obstacles=obstacles, image_file="/home/nermin/RPMPLv2/data/xarm6/xarm6.gif", is_trajectory=True, fps=10.0)
    # path = [1.5708, 1.5708, -2.3562, 0, 0, 0]
    # visualize(path, obstacles=obstacles, image_file=None, is_trajectory=False, fps=10.0)
    