import os
os.environ["PYOPENGL_PLATFORM"] = "egl"

from trimesh.creation import box
from planar_2dof.planar_2dof import Planar2DOF
from math import pi
import time

obstacles = [box([0.2, 1.2, 0.1])]
obstacles[0].apply_translation([1.3+0.25, 0, 0])

planar_2dof = Planar2DOF(obstacles)
robot = planar_2dof.robot


for link in robot.actuated_joints:
    print(link.name)

print("Number of joints: ", len(robot.actuated_joints))

robot.show( [-pi/4, pi/6], obstacles )
