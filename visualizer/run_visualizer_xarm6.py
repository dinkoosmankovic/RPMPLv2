from log_parser import LogParser
from xarm6_vis import visualize
import yaml
import argparse


if __name__ == "__main__":
    parser = LogParser("/tmp/plannerData.log")
    path = parser.get_path()
    
    arg_parser = argparse.ArgumentParser(description='Processing visualizer input argumetns')
    arg_parser.add_argument('-s', '--scenario', default='../data/xarm6/scenario_easy.yaml', help='Scenario file path')
    
    args = arg_parser.parse_args()
    print(args.scenario)
    #for p in path:
    #    print(p)
    #print(path)
    #visualize(path[0], "test.png")
    with open(args.scenario, 'r') as file:
        obstacles = yaml.safe_load(file)
    visualize(path, obstacles=obstacles, image_file=None, is_trajectory=True, fps=10.0)
    # path = [1.5708, 1.5708, -2.3562, 0, 0, 0]
    # visualize(path, obstacles=obstacles, image_file=None, is_trajectory=False, fps=10.0)
    