import rclpy
from rclpy.node import Node
import roboticstoolbox as rtb

import numpy as np
from spatialmath  import SE3, SO3

import time
from threading import Thread, Event
import copy

from vive_tracker_client import ViveTrackerClient
from franka_state_interface import FrankaStateInterface

from queue import Queue

import cProfile, pstats, io
from pstats import SortKey
pr = cProfile.Profile()
pr.enable()


class ViveRecieverThread():
    def __init__(self, host, port, tracker_name):
        self.client = ViveTrackerClient(host=host,
                                        port=port,
                                        tracker_name=tracker_name,
                                        should_record=False)
        self.kill_thread = Event()
        self.client_thread = Thread(target=self.client.run_threaded, args=(self.kill_thread,))

        self.client_thread.start()

    def get_data(self):
        return self.client.latest_tracker_message

    def kill(self):
        self.kill_thread.set()
        self.client_thread.join()


class TrajPublisher:
    def __init__(self, fsi):
        self.fsi = fsi
        self.queue = Queue(maxsize=20)
        self.kill_thread = Event()
        self.traj_thread = Thread(target=self.publisher_thread)

        self.traj_thread.start()

    def publisher_thread(self):
        while rclpy.ok():
            joint_solutions = self.queue.get()
            print(joint_solutions)
            self.fsi.publish_joints(joint_solutions)
            time.sleep(0.005)
    
    def add_traj(self, traj: list[list]):
        for joint_sol in traj:
            self.queue.put(joint_sol)


def main(args=None):
    rclpy.init(args=args)
    node_handle = Node('teleoperation_node')

    fsi = FrankaStateInterface(node_handle)
    # traj_publisher = TrajPublisher(fsi)

    spin_func = lambda _ : rclpy.spin(node_handle)
    spin_thread = Thread(target=spin_func, args=(0,))
    spin_thread.start()
    time.sleep(1) # sleep to allow spin thread to get some messages
    panda = rtb.models.DH.Panda()
    panda.tool = SE3()

    # print(fsi.joint_positions)
    initial_joint_pos = np.array(fsi.joint_positions)
    desired_transform_se3 =  panda.fkine(initial_joint_pos)
    current_joint_positions = copy.deepcopy(initial_joint_pos)
    joint_solutions = copy.deepcopy(current_joint_positions)

    velocity_scale = 0.3 # m
    rot_vel_scale = 0.5 # deg

    calibration_transformation = np.eye(3)
    
    try:
        HOST, PORT = "192.168.1.103", 8000
        controller = ViveRecieverThread(HOST, PORT, "controller_1")
        # tracker = ViveRecieverThread(HOST, PORT, "tracker_1")
        controller_data = controller.get_data()
        last_pos = None

        while rclpy.ok():
            controller_data = controller.get_data()
            if controller_data is None:
                last_pos = None
                print("Controller not found")
                time.sleep(1)
                continue
            if last_pos is None:
                last_pos = np.array([controller_data.x, controller_data.y, controller_data.z])

            controller_vel = np.array([controller_data.x, 
                                       controller_data.y, 
                                       controller_data.z]) - last_pos

            controller_rot_vel = [controller_data.p, 
                                  controller_data.q, 
                                  controller_data.r]

            if controller_data.menu_button == 1:
                calibration_transformation = controller_data.rotation_as_scipy_transform().as_matrix()
                print(calibration_transformation)
            # elif controller_data.trigger == 1:
            elif True:
                cartesian_increment =  np.matmul(calibration_transformation, np.array([
                    -controller_vel[0] * velocity_scale,
                    controller_vel[2] * velocity_scale, 
                    controller_vel[1] * velocity_scale,
                    ]))
                
                rpy_increment = np.matmul(calibration_transformation, np.array([
                    -controller_rot_vel[1] * rot_vel_scale, 
                    -controller_rot_vel[2] * rot_vel_scale,
                    controller_rot_vel[0] * rot_vel_scale, 
                    ]))
                
                desired_transform_se3 = SE3(cartesian_increment) * desired_transform_se3 * SE3.RPY(rpy_increment, order='xyz', unit="deg")
                # desired_transform_se3 = SE3(cartesian_increment) * desired_transform_se3 
                
                sol = panda.ikine_LM(desired_transform_se3, q0=current_joint_positions)

                if sol.success:
                    print("controller_rot_vel", controller_rot_vel)
                    print("controller_vel", controller_vel)
                    print("current_joint_positions", current_joint_positions)
                    joint_solutions = sol.q
                    current_joint_positions = sol.q
                    print("joint_solutions", joint_solutions)
                    print("desired_transform_se3", desired_transform_se3)
                    # traj_publisher.add_traj([joint_solutions.tolist()])
                    # fsi.publish_joints(joint_solutions.tolist())
            
            last_pos = np.array([controller_data.x, controller_data.y, controller_data.z])
    finally:
        pr.disable()
        s = io.StringIO()
        sortby = SortKey.TIME
        ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
        ps.print_stats(20)
        print(s.getvalue())
        controller.kill()
        # tracker.kill()


if __name__ == "__main__":
    main()