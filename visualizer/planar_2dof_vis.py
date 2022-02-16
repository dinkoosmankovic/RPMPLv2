#import os
#os.environ["PYOPENGL_PLATFORM"] = "osmesa"

from trimesh.creation import box
from planar_2dof.planar_2dof import Planar2DOF
from math import pi
import time

def visualize(q=None, obstacles=None, image_file=None, is_trajectory=False, fps=10.0):
    #obstacles = [box([0.2, 1.2, 0.1])]
    #obstacles[0].apply_translation([1.3+0.25, 0, 0])
    print(obstacles)
    obstacles = obstacles['obstacles']
    for i, obs in enumerate(obstacles):
        obs = obs['box']
        dim = obs['dim']
        trans = obs['trans']
        obstacles[i] = box(dim)
        obstacles[i].apply_translation(trans)

    planar_2dof = Planar2DOF(obstacles)
    robot = planar_2dof.robot
    for link in robot.actuated_joints:
        print(link.name)
    print("Number of joints: ", len(robot.actuated_joints))

    if not is_trajectory:
        planar_2dof.show( q, obstacles, image_file)
    else:
        planar_2dof.animate(q, obstacles=obstacles, fps=fps, image_file=image_file)
