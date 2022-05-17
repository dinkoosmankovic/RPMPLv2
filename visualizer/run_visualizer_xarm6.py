from log_parser import LogParser
from xarm6_vis import visualize
import yaml
import argparse


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser(description='Processing visualizer input argumetns')
    arg_parser.add_argument('-s', '--scenario', default='../data/xarm6/scenario_easy.yaml', help='Scenario file path')
    arg_parser.add_argument('-l', '--log_path', default='/tmp/plannerData.log', help='Log file path')
    args = arg_parser.parse_args()
    
    parser = LogParser(args.log_path)
    path = parser.get_path()    

    with open(args.scenario, 'r') as file:
        obstacles = yaml.safe_load(file)
    visualize(path, obstacles=obstacles, image_file=None, is_trajectory=True, fps=10.0)
    # path = [1.5708, 1.5708, -2.3562, 0, 0, 0]
    # visualize(path, obstacles=obstacles, image_file=None, is_trajectory=False, fps=10.0)
    