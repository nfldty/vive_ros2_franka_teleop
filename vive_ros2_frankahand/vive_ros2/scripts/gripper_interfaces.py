import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from franka_msgs.action import Grasp, Homing, Move
from std_srvs.srv import Trigger

# ------- Copied from rclpy source code and modified --------
class TimeoutObject:
    """Use timeout object to save timeout."""

    def __init__(self, timeout: float) -> None:
        self._timeout = timeout

    @property
    def timeout(self) -> float:
        return self._timeout

    @timeout.setter
    def timeout(self, timeout: float) -> None:
        self._timeout = timeout

# copied from rclpy.spin_until_future_complete, remove spins, and add a hardcoded 0.001 sleep
def wait_until_future_complete(
    future,
    timeout_sec: Optional[float] = None # None will wait indefinitely
) -> None:

    if timeout_sec is None or timeout_sec < 0:
        while (not future.done() and not future.cancelled()):
            time.sleep(0.001) # 100hz
    else:
        start = time.monotonic()
        end = start + timeout_sec
        timeout_left = TimeoutObject(timeout_sec)
        while (not future.done() and not future.cancelled()):
            time.sleep(0.001) # 100hz
            now = time.monotonic()
            if now >= end:
                return
            timeout_left.timeout = end - now
# -------------------------------------------------------

# This class assumes you have a spin thread running outside of it
class FrankaGripperActionClient():
    def __init__(self, node_handle):
        self._homing_action_client = ActionClient(node_handle, Homing, '/fr3_gripper/homing')
        self._move_action_client = ActionClient(node_handle, Move, '/fr3_gripper/move')
        self._grasp_action_client = ActionClient(node_handle, Grasp, '/fr3_gripper/grasp')
        self._cancel_action_server_client = node_handle.create_client(Trigger, '/fr3_gripper/stop')
        # while not self._cancel_action_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('cancel action service not available, waiting again...')        
        self.cancel_action_req = Trigger.Request()

    def _action_blocking_helper(self, future):
        wait_until_future_complete(future)
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        wait_until_future_complete(result_future)
        return result_future.result()

    def do_homing_async(self):
        homing_msg = Homing.Goal()
        self._homing_action_client.wait_for_server()
        return self._homing_action_client.send_goal_async(homing_msg)

    def do_homing_blocking(self):
        future = self.do_homing_async()
        return self._action_blocking_helper(future)

    def do_move_async(self, width, speed):
        move_msg = Move.Goal() 
        move_msg.width = width
        move_msg.speed = speed
        self._move_action_client.wait_for_server() 
        return self._move_action_client.send_goal_async(move_msg)

    def do_move_blocking(self, width, speed):
        future = self.do_move_async(width, speed)
        return self._action_blocking_helper(future)

    def do_grasp_async(self, width, speed, force = 50.0, epsilon=0.005): 
        grasp_msg = Grasp.Goal()
        grasp_msg.width = width 
        grasp_msg.speed = speed 
        grasp_msg.force = force
        grasp_msg.epsilon.inner = epsilon
        grasp_msg.epsilon.outer = epsilon
        self._grasp_action_client.wait_for_server()
        return self._grasp_action_client.send_goal_async(grasp_msg)

    def do_grasp_blocking(self, width, speed, force = 50.0):
        future = self.do_grasp_async(width, speed, force)
        return self._action_blocking_helper(future)

    def cancel_action_async(self): # -> Future
        return self._cancel_action_server_client.call_async(self.cancel_action_req)
    
    def cancel_action_blocking(self):
        future = self.cancel_action_async()
        wait_until_future_complete(future)
        return future.result()