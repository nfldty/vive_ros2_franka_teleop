import threading
import rclpy
from rclpy.node import Node
from gripper_interfaces import FrankaGripperActionClient, wait_until_future_complete

def main(args=None):
    rclpy.init(args=args)
    node_handle = Node('joint_control')

    franka_gripper_client = FrankaGripperActionClient(node_handle)

    spin_func = lambda _ : rclpy.spin(node_handle)
    spin_thread = threading.Thread(target=spin_func, args=(0,))
    spin_thread.start()

    # ASYNC calls
    print("calling async functions")
    future = franka_gripper_client.do_homing_async() 
    wait_until_future_complete(future) 
    print("send homing goal result", future.result())

    future = franka_gripper_client.do_move_async(width=0.01, speed=0.2)
    wait_until_future_complete(future) 
    print("send move result", future.result())

    future = franka_gripper_client.do_grasp_async(width=0.05, speed=0.2)
    wait_until_future_complete(future) 
    goal_handle = future.result()
    print("send grasp result", goal_handle)

    result_future = goal_handle.get_result_async()
    wait_until_future_complete(result_future) 
    print("Finished grasping")

    # SYNC
    print("calling blocking functions")
    print("homing...")
    franka_gripper_client.do_homing_blocking()
    print("finished homing...")

    print("moving...")
    franka_gripper_client.do_move_blocking(width=0.01, speed=0.2)
    print("finished moving...")

    print("grasping...")
    franka_gripper_client.do_grasp_blocking(width=0.05, speed=0.2)
    print("finished grasping...")

    # franka_gripper_client.cancel_action()

if __name__ == '__main__':
    main()