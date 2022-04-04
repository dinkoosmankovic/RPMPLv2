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
    with open('../data/xarm6/scenario2.yaml', 'r') as file:
        obstacles = yaml.safe_load(file)
    visualize(path, obstacles=obstacles, image_file=None, is_trajectory=True, fps=10.0)
    # path = [1.49469,   1.81164,  -3.35258,  0.626211, -0.117099, -0.466912]
    # visualize(path, obstacles=obstacles, image_file=None, is_trajectory=False, fps=10.0)
    