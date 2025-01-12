from Direction import AbsoluteDirection, Direction
from json import *
import os

class HumanPath:

    def __init__(self, speed=0.1):
        self.direction_list = []
        self.speed = speed

    def at(self, index):
        return self.direction_list[index]
    
    def add_path(self, path):
        self.direction_list.append((path, self.speed))

    def parse(self, dir_path):
        with open(dir_path, 'r', encoding="utf-8") as file_data:
            file_content = file_data.read()
            data = loads(file_content)

            root_rel_direction = Direction(x_vec=0.0, y_vec=0.0, z_vec=0.0)
            root_dir = AbsoluteDirection(rel_direction=root_rel_direction, current_xyz=(data["points"][0][0], data["points"][0][1], 0))

            self.add_path(root_dir)
            

            for i in range(1, len(data["points"])):
                temp_rel_direction = Direction(x_vec=data["points"][i][0] - data["points"][i - 1][0], y_vec=data["points"][i][1] - data["points"][i - 1][1])
                abs_dir = AbsoluteDirection(rel_direction=temp_rel_direction, current_xyz=(data["points"][i][0], data["points"][i][1], 0))
                self.add_path(abs_dir)
            
"""
def test():
    hp = HumanPath()

    with open('./src/test_path.json', 'r', encoding="utf-8") as file_data:
        file_content = file_data.read()  # 한 번만 호출
        print(file_content)  # 파일 내용 출력
        data = loads(file_content)  # JSON 파싱
        print(data)

        print(data)

        root_rel_direction = Direction(x_vec=0.0, y_vec=0.0, z_vec=0.0)
        root_dir = AbsoluteDirection(rel_direction=root_rel_direction, current_xyz=(data["points"][0][0], data["points"][0][1], 0))

        hp.add_path(root_dir)
        

        for i in range(1, len(data["points"])):
            temp_rel_direction = Direction(x_vec=data["points"][i][0] - data["points"][i - 1][0], y_vec=data["points"][i][1] - data["points"][i - 1][1])
            abs_dir = AbsoluteDirection(rel_direction=temp_rel_direction, current_xyz=(data["points"][i][0], data["points"][i][1], 0))
            hp.add_path(abs_dir)
        

        print(hp.direction_list)

if __name__ == "__main__":
    pass 
    #test()

"""