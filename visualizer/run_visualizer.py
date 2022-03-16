from log_parser import LogParser
from planar_2dof_vis import visualize
import yaml


if __name__ == "__main__":
    # parser = LogParser("/tmp/plannerData.log")
    # path = parser.get_path()
    #for p in path:
    #    print(p)
    #print(path)
    #visualize(path[0], "test.png")
    # with open('../data/planar_2dof/scenario_easy.yaml', 'r') as file:
    with open('../data/planar_2dof/scenario1.yaml', 'r') as file:
        obstacles = yaml.safe_load(file)     
    path = [-0.380458,  0.582361]
    visualize(path, obstacles=obstacles, image_file=None, is_trajectory=False, fps=10.0)
    
