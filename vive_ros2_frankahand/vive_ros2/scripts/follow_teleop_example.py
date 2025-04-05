import numpy as np
import time
from scipy.spatial.transform import Rotation as R

def get_rot_and_p(tf):
    rot = tf[0:3, 0:3]
    p = tf[0:3, 3]
    return rot, p.flatten('F')

def get_scaled_rotation(rotation, scale):
    t = scale  # Interpolation parameter (from 0 to 1)
    scaled_quat = R.slerp([0, 1], [R.identity(), R.from_matrix(rotation)])(t)
    return scaled_quat.as_matrix()

class TeleopController():
    def __init__(self):
        self.is_registered = False
        self.is_clutched = False
        self.is_enabled = False

    def clutch(self):
        self.is_clutched = True

    def unclutch(self):
        self.is_clutched = False

    def enable(self):
        self.is_enabled = True

    def disable(self):
        self.is_enabled = False

    def check_state_enabled(self):
        if(not self.is_registered):
            print("Hey, need to call register. I'm not updating")
            return False
        if(self.is_clutched):
            return False
        if(not self.is_enabled):
            return False
        return True

# This is for your robot if your robot tip transform change
# matches the tip transform change of the input device
class CartesianFollowTeleopController(TeleopController):
    def __init__(self, kinematics_solver,
                 position_scale = 1.0,
                 rotation_scale = 1.0):
        super().__init__()
        self.kinematics_solver = kinematics_solver

        # State variables
        self.current_output_tf = np.identity(4)
        self.current_output_js = np.array([])
        self.start_input_tf = np.identity(4)
        self.start_output_tf = np.identity(4)

        self.position_scale = position_scale
        self.rotation_scale = rotation_scale

    # start_input_tf: np.array 4x4 matrix of the current transform of the input device
    # start_output_js: np.array n joints of the current joint position of the output device/robot
    def enable(self, start_input_tf, start_output_js):
        self.start_input_tf = np.matmul(start_input_tf, self.input_tf_appended_rotation)
        self.current_output_js = np.copy(start_output_js)
        self.start_output_tf = np.copy(self.kinematics_solver.compute_fk(start_output_js))
        self.current_output_tf = np.copy(self.start_output_tf)
        super().enable()

    # start_input_tf: np.array 4x4 matrix of the current transform of the input device
    # start_output_js: np.array n joints of the current joint position of the output device/robot
    def unclutch(self, start_input_tf, start_output_js):
        self.start_input_tf = np.matmul(start_input_tf, self.input_tf_appended_rotation)
        self.current_output_js = np.copy(start_output_js)
        self.start_output_tf = np.copy(self.kinematics_solver.compute_fk(start_output_js))
        super().unclutch()


    def output_callback(new_joint_positions):
        # Publish here you new joint states here
        pass

    # absolute_input_tf: continuously update the input devices 4x4 transform matrix
    def update(self, absolute_input_tf):
        if(not self.check_state_enabled()):
            return False

        absolute_output_tf = self.__calculate_output_tf(absolute_input_tf)
        new_output_js = self.kinematics_solver.compute_ik(absolute_output_tf, self.current_output_js)
        if np.isnan(new_output_js).any():
            print("Controller: IK solution isnan returning false!! ")
            print("Controller: ik_solution: ", new_output_js)
            return False
        self.current_output_js = new_output_js
        self.output_callback(self.current_output_js)
        return True

    def __calculate_output_tf(self, absolute_input_tf):
        absolute_input_tf = np.matmul(absolute_input_tf, self.input_tf_appended_rotation)
        input_tf_rot_diff = np.matmul(np.transpose(self.start_input_tf[0:3, 0:3]), absolute_input_tf[0:3, 0:3])

        input_tf_difference = np.identity(4)
        input_tf_difference[0:3,0:3] = input_tf_rot_diff
        input_tf_difference[0:3, 3] = (absolute_input_tf[0:3,3] - self.start_input_tf[0:3,3]) * self.position_scale

        absolute_output_tf = np.copy(self.start_output_tf)
        absolute_output_tf = self.__update_output_tf(input_tf_difference, absolute_output_tf)
        self.current_output_tf = absolute_output_tf
        return absolute_output_tf


    def __update_output_tf(self, input_diff_tf, current_output_tf):
        input_diff_rot, input_diff_p = get_rot_and_p(input_diff_tf)
        cur_output_rot, cur_output_p = get_rot_and_p(current_output_tf)

        output_diff_p_wrt_s = input_diff_p
        output_p_wrt_s = cur_output_p + output_diff_p_wrt_s

        input_diff_rot_scaled = get_scaled_rotation(input_diff_rot, self.rotation_scale)
        # Post multiply output to rotate about itself
        output_rot = np.matmul(cur_output_rot, input_diff_rot_scaled)

        output_tf = np.copy(current_output_tf)
        output_tf[0:3, 0:3] = output_rot
        output_tf[0:3, 3] = output_p_wrt_s
        return output_tf

def interpolate_joint_path(start_jp, goal_jp, increment):
    joint_path = np.arange(start_jp, goal_jp, increment)
    joint_path = np.append(joint_path, goal_jp)
    return joint_path

# This is for your Gripper if the joint state needs to match an input device joint state
class JointFollowTeleopController(TeleopController):
    def __init__(self, scale = 1.0):
        super().__init__()
        self.scale = scale

    def output_callback(new_joint_positions):
        # Publish here you new joint states here
        pass

    def execute_on_output_callback(self, joint_path, joint_state, idx, hz):
        joint_state_copy = np.copy(joint_state)
        for jp in joint_path:
            joint_state_copy[idx] = jp
            self.output_callback(joint_state_copy)
            time.sleep(1.0/hz)
        return joint_state_copy

    def execute_match_joint_states(self, joint_state, match_to_joint_state):
        updated_output_jps = np.copy(joint_state)
        joint_too_far_tol = 0.1
        for i in range(len(match_to_joint_state)): # If multiple joints, usually its just one
            if abs(match_to_joint_state[i] - updated_output_jps[i]) > joint_too_far_tol:
                joint_path_inc = interpolate_joint_path(
                    updated_output_jps[i],
                    match_to_joint_state[i],
                    joint_too_far_tol/4)
                updated_output_jps = self.execute_on_output_callback(
                        joint_path_inc, updated_output_jps, i, 50)

    def enable(self, start_input_jps, start_output_jps):
        scaled_input_jps = self.scale * np.array(start_input_jps)
        self.execute_match_joint_states(start_output_jps, scaled_input_jps)
        super()._enable()

    def unclutch(self, start_input_jps, start_output_jps):
        scaled_input_jps = self.scale * np.array(start_input_jps)
        self.execute_match_joint_states(start_output_jps, scaled_input_jps)
        super()._unclutch()

