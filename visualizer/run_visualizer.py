from log_parser import LogParser




if __name__ == "__main__":
    parser = LogParser("/home/dinko/RPMPLv2/visualizer/plannerData.log")
    path = parser.get_path()
    for p in path:
        print(p)
