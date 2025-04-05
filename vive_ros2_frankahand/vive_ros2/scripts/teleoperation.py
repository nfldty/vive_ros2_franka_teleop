import copy
import numpy as np
import pygame
import threading
import time

from spatialmath  import SE3
from spatialmath.base import *
import roboticstoolbox as rtb
import rclpy
from rclpy.node import Node

from interfaces import FrankaStateInterface


def main(args=None):
    rclpy.init(args=args)
    node_handle = Node('teleoperation_node')
    fsi = FrankaStateInterface(node_handle)

    spin_func = lambda _ : rclpy.spin(node_handle)
    spin_thread = threading.Thread(target=spin_func, args=(0,))
    spin_thread.start()
    time.sleep(2) # sleep to allow spin thread to get some messages

    pygame.init()
    screen = pygame.display.set_mode((640, 200))
    pygame.display.set_caption("Click and Press Keys to Teleop")
    pygame.mouse.set_visible(1)    

    panda = rtb.models.Panda()
    panda.tool = SE3()

    initial_joint_pos = np.array(fsi.joint_positions)

    # Task 1: Measure the tool offset from the flange
    tool_offset = np.array([0, 0, 0.145])
    desired_transform_se3 =  panda.fkine(initial_joint_pos) * SE3(tool_offset)
    current_joint_positions = copy.deepcopy(initial_joint_pos)

    velocity = 0.004 # m
    rot_vel = 0.006 # rad

    joint_solutions = copy.deepcopy(current_joint_positions)

    while (rclpy.ok()):
        cartesian_increment = np.array([0.0, 0.0, 0.0])
        rpy_increment = np.array([0.0, 0.0, 0.0])
        events = pygame.event.get()
        keys = pygame.key.get_pressed()
            # if event.type == pygame.KEYDOWN:
        if keys[pygame.K_LEFT]:
            cartesian_increment[0] -= 1 * velocity
        if keys[pygame.K_RIGHT]:
            cartesian_increment[0] += 1 * velocity
        if keys[pygame.K_UP]:
            cartesian_increment[1] -= 1 * velocity
        if keys[pygame.K_DOWN]:
            cartesian_increment[1] += 1 * velocity
        if keys[pygame.K_b]:
            cartesian_increment[2] -= 1 * velocity
        if keys[pygame.K_t]:
            cartesian_increment[2] += 1 * velocity

        if keys[pygame.K_w]:
            rpy_increment[0] -= 1 * rot_vel
        if keys[pygame.K_s]:
            rpy_increment[0] += 1 * rot_vel
        if keys[pygame.K_a]:
            rpy_increment[1] -= 1 * rot_vel
        if keys[pygame.K_d]:
            rpy_increment[1] += 1 * rot_vel
        if keys[pygame.K_q]:
            rpy_increment[2] -= 1 * rot_vel
        if keys[pygame.K_e]:
            rpy_increment[2] += 1 * rot_vel

        # print("cartesian_increment", cartesian_increment)
        # print("rpy_increment", rpy_increment)
        
        
        # Task: Control the robot to move tooltip in the world frame(fixed frame), and
        # rotate in end effector’s local frame(body frame)
        desired_transform_se3 = SE3(cartesian_increment) * desired_transform_se3 #* SE3.RPY(rpy_increment, order='xyz')
        
        back_prop = np.array([0,0,-0.145])      
        desired_transform_se3_flange = desired_transform_se3 * SE3(back_prop)

        # Task: Solve the joint positions of this new transform using robotics toolbox panka IK (remember to set q0)
        sol = panda.ikine_LM(desired_transform_se3_flange, q0=current_joint_positions)
        
        if sol.success:
            joint_solutions = sol.q
            current_joint_positions = sol.q
        # print("desired_transform_se3", desired_transform_se3)
        # print("initial_joint_pos", initial_joint_pos)
            print("joint_solutions", joint_solutions)
            fsi.publish_joints(joint_solutions.tolist())
        time.sleep(0.01)
        
    spin_thread.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

