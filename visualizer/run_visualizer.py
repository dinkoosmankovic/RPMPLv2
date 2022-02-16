from log_parser import LogParser
from planar_2dof_vis import visualize



if __name__ == "__main__":
    parser = LogParser("/home/dinko/RPMPLv2/visualizer/plannerData.log")
    path = parser.get_path()
    #for p in path:
    #    print(p)
    #print(path)
    #visualize(path[0], "test.png")
    visualize(path, "test.gif", is_trajectory=True, fps=10.0)
    
