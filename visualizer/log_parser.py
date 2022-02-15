import re
from copy import deepcopy

class LogParser:
    def __init__(self, file_name=None) -> None:
        if file_name is None:
            raise RuntimeError("No filename provided!")
        
        with open(file_name) as file:
            self.parse_file(file)

    def parse_file(self, file=None):
        if file is None:
            return
        
        line_nums_path = self.search_string_in_file(deepcopy(file), "Path:")
        print(line_nums_path)       
        file.close()

    def search_string_in_file(self, file, string_to_search):
        line_number = 0
        list_of_results = []
        for line in file:
            line_number += 1
            if string_to_search in line:
                list_of_results.append((line_number, line.rstrip()))
        return list_of_results

    def _parse_line(self, line):
        pass



if __name__ == "__main__":
    parser = LogParser("/home/dinko/RPMPLv2/visualizer/plannerData.log")




        

