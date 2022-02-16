from urdfpy import URDF
import pyrender
from state_spaces.real_vector_space import RealVectorSpace
from trimesh.creation import box, cylinder
from trimesh.collision import CollisionManager
import numpy as np
import threading


class Planar2DOF(RealVectorSpace):
    def __init__(self, obstacles) -> None:
        self.robot = URDF.load('../data/planar_2dof/planar_2dof.urdf')
        self.spaces = RealVectorSpace(2)
        self.robot_cm = CollisionManager()
        self.env_cm = CollisionManager()
        self.start_config = [0, 0]

        self.traj = []
        self.count_i = 0
        self.curr_q = [0, 0]

        cfg = self.get_config([0, 0])
        fk = self.robot.link_fk(cfg=cfg)
        self.init_poses = []
        for i, tm in enumerate(fk):
            if i == 3:
                break
            pose = fk[tm]
            init_pose, link_mesh = self.get_link_mesh(tm)
            self.init_poses.append(init_pose)
            self.robot_cm.add_object(
                tm.name, link_mesh, np.matmul(pose, init_pose))

        for i, ob in enumerate(obstacles):
            self.env_cm.add_object("obstacle_" + str(i), ob)
            self.env_cm.add_object("prediction_" + str(i), ob)

    def set_trajectory(self, path):
        self.traj = path
        self.curr_q = path[0]
        self.count_i = 0

    def get_next_q(self):
        self.count_i += 1
        if self.count_i >= len(self.traj):
            return None

        self.curr_q = self.traj[self.count_i]
        return self.curr_q

    def update_env(self, obstacles):
        with threading.Lock():
            for i, ob in enumerate(obstacles):
                self.env_cm.remove_object("obstacle_" + str(i))
                self.env_cm.add_object("obstacle_" + str(i), ob)

    def update_predictions(self, predictions):
        with threading.Lock():
            for i, ob in enumerate(predictions):
                self.env_cm.remove_object("prediction_" + str(i))
                self.env_cm.add_object("prediction_" + str(i), ob)

    def spaces(self):
        return self.spaces

    def robot(self):
        return self.robot

    def is_in_collision(self, q):
        cfg = self.get_config(q)
        fk = self.robot.link_fk(cfg=cfg)
        # adding robot to the scene
        for i, tm in enumerate(fk):
            if i == 3:
                break
            pose = fk[tm]
            init_pose = self.init_poses[i]
            self.robot_cm.set_transform(tm.name, np.matmul(pose, init_pose))

        # print("obs:", self.env_cm._objs["obstacle_0"]["obj"].getTransform())
        return self.robot_cm.in_collision_other(self.env_cm)

    def distance(self, q):
        cfg = self.get_config(q)
        fk = self.robot.link_fk(cfg=cfg)
        # adding robot to the scene
        for i, tm in enumerate(fk):
            pose = fk[tm]
            init_pose = self.init_poses[i]
            self.robot_cm.set_transform(tm.name, np.matmul(pose, init_pose))

        # print("obs:", self.env_cm._objs["obstacle_0"]["obj"].getTransform())
        return self.robot_cm.min_distance_other(self.env_cm)

    def is_valid(self, q, qe=None, num_checks=None):
        if qe is None or num_checks is None:
            res = self.is_in_collision(q)
            return not(res)
        else:
            for i in range(num_checks):
                q_i = self.get_qnew(q, qe, i/num_checks)
                res = self.is_in_collision(q_i)
                if res:
                    return False
        return True

    def get_config(self, q):
        if len(self.robot.actuated_joints) != len(q):
            raise Exception("Wrong dimensions of q")

        cfg = {}
        for i in range(len(q)):
            cfg[self.robot.actuated_joints[i].name] = q[i]

        return cfg

    def get_link_mesh(self, tm):
        init_pose = tm.visuals[0].origin
        if tm.visuals[0].geometry.cylinder is not None:
            length = tm.visuals[0].geometry.cylinder.length
            radius = tm.visuals[0].geometry.cylinder.radius
            mesh = cylinder(radius, length)
            mesh.visual.face_color = tm.visuals[0].material.color
            return init_pose, mesh
        else:
            ext = tm.visuals[0].geometry.box.size
            mesh = box(ext)
            mesh.visual.face_colors = tm.visuals[0].material.color
            return init_pose, mesh

    def show(self, q=None, obstacles=None):
        print("showing")
        cfg = self.get_config(q)
        print(cfg)

        fk = self.robot.link_fk(cfg=cfg)

        scene = pyrender.Scene()
        # adding robot to the scene
        for i, tm in enumerate(fk):
            if i == 3:
                break
            pose = fk[tm]
            init_pose, link_mesh = self.get_link_mesh(tm)
            mesh = pyrender.Mesh.from_trimesh(link_mesh, smooth=False)
            scene.add(mesh, pose=np.matmul(pose, init_pose))

        # adding base box to the scene
        # table = box([0.5, 0.5, 0.01])
        # table.apply_translation([0, 0, -0.1])

        cam = pyrender.PerspectiveCamera(yfov=np.pi / 3.0, aspectRatio=1.414)
        # nc = pyrender.Node(camera=cam, matrix=np.eye(4))
        # scene.add_node(nc)
        init_cam_pose = np.eye(4)
        init_cam_pose[2, 3] = 2.5
        scene.add(cam, pose=init_cam_pose)

        # scene.add(pyrender.Mesh.from_trimesh(table))

        # adding obstacles to the scene
        for ob in obstacles:
            scene.add(pyrender.Mesh.from_trimesh(ob, smooth=False))

        pyrender.Viewer(scene, use_raymond_lighting=True)
        #r = pyrender.OffscreenRenderer(viewport_width=640*2, viewport_height=480*2)
        #color, depth = r.render(scene)
        #r.delete()
        #print("showing")
        #import matplotlib.pyplot as plt
        #plt.figure(figsize=(20,20))
        #plt.imshow(color)
        #plt.show()

    def animate(self, q_traj=None, obstacles=None):
        import time
        fps = 10.0
        cfgs = [self.get_config(q) for q in q_traj]

        # Create the scene
        fk = self.robot.link_fk(cfg=cfgs[0])

        node_map = {}
        init_pose_map = {}
        scene = pyrender.Scene(ambient_light=[0.02, 0.02, 0.02], bg_color=[1.0, 1.0, 1.0])
        for tm in fk:
            pose = fk[tm]
            init_pose, link_mesh = self.get_link_mesh(tm)
            mesh = pyrender.Mesh.from_trimesh(link_mesh, smooth=False)
            node = scene.add(mesh, pose=np.matmul(pose, init_pose))
            node_map[tm] = node
            init_pose_map[tm] = init_pose

        cam = pyrender.PerspectiveCamera(yfov=np.pi / 3.0, aspectRatio=1.414)
        init_cam_pose = np.eye(4)
        init_cam_pose[2, 3] = 2.5
        scene.add(cam, pose=init_cam_pose)

        for ob in obstacles:
            scene.add(pyrender.Mesh.from_trimesh(ob, smooth=False))

        # Pop the visualizer asynchronously
        v = pyrender.Viewer(scene, run_in_thread=True,
                            use_raymond_lighting=True)
        time.sleep(1.0)
        # Now, run our loop
        i = 0
        while v.is_active:
            cfg = cfgs[i]
            fk = self.robot.link_fk(cfg=cfg)
            if i < len(cfgs) - 1:
                i += 1
            else:
                i = 0
                time.sleep(1.0)
            v.render_lock.acquire()
            for mesh in fk:
                pose = fk[mesh]
                node_map[mesh].matrix = np.matmul(pose, init_pose_map[mesh])
            v.render_lock.release()
            time.sleep(1.0 / fps)
