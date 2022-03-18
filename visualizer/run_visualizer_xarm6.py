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
    # with open('../data/xarm6/scenario_easy.yaml', 'r') as file:
    with open('../data/xarm6/scenario3.yaml', 'r') as file:
        obstacles = yaml.safe_load(file)
    # path = [-0.750574,  0.868776, -0.768613,  0.705666,  0.190886, -0.229982];
    visualize(path, obstacles=obstacles, image_file="easy_xarm6.gif", is_trajectory=True, fps=10.0)
    
