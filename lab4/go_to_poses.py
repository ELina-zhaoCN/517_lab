#!/usr/bin/env python3
"""
Execute a sequence of saved end-effector poses with gripper commands.

Loads poses from a CSV file (saved by save_pose.py) and executes a
pre-defined sequence interleaving pose goals and gripper open/close commands.

Prerequisites:
    # 1. Launch MoveIt:
    ros2 launch soa_moveit_config soa_moveit_bringup.launch.py

    # 2. Start action servers:
    ros2 run soa_functions move_to_pose_server
    ros2 run soa_functions gripper_server

    # 3. Run this app:
    ros2 run soa_apps go_to_poses --ros-args -p csv_path:=/path/to/poses.csv
"""

import csv

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose
from soa_interfaces.action import Gripper, MoveToPose


DEFAULT_CSV_PATH = '/home/ubuntu/techin517/poses.csv'

GRIPPER_OPEN = 1.7453
GRIPPER_CLOSED = 0.1

# Sequence of steps: ('pose', index) or ('gripper', position)
SEQUENCE = [
    ('pose', 0),
    ('gripper', GRIPPER_OPEN),
    ('pose', 1),
    ('gripper', GRIPPER_CLOSED),
    ('pose', 2),
    ('pose', 3),
    ('gripper', GRIPPER_OPEN),
]


def load_poses(path: str) -> list:
    """Load saved poses from a CSV file into a list of Pose objects."""
    poses = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            pose = Pose()
            pose.position.x = float(row['pos_x'])
            pose.position.y = float(row['pos_y'])
            pose.position.z = float(row['pos_z'])
            pose.orientation.x = float(row['ori_x'])
            pose.orientation.y = float(row['ori_y'])
            pose.orientation.z = float(row['ori_z'])
            pose.orientation.w = float(row['ori_w'])
            poses.append(pose)
    return poses


class GoToPoses(Node):

    def __init__(self):
        super().__init__('go_to_poses')

        self.declare_parameter('csv_path', DEFAULT_CSV_PATH)

        self._pose_client = ActionClient(self, MoveToPose, 'move_to_pose')
        self._gripper_client = ActionClient(self, Gripper, 'gripper_command')

    def send_pose_goal(self, pose: Pose) -> bool:
        """Use the MoveToPose action client to move the arm to a target pose."""
        goal = MoveToPose.Goal()
        goal.target_pose = pose

        self.get_logger().info(
            f'Sending pose goal: pos=({pose.position.x:.3f}, '
            f'{pose.position.y:.3f}, {pose.position.z:.3f})'
        )

        self._pose_client.wait_for_server()
        future = self._pose_client.send_goal_async(
            goal, feedback_callback=self._pose_feedback_callback)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Pose goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'Pose goal succeeded: {result.message}')
        else:
            self.get_logger().error(f'Pose goal failed: {result.message}')
        return result.success

    def send_gripper_goal(self, target_position: float) -> bool:
        """Use the Gripper action client to move the gripper."""
        goal = Gripper.Goal()
        goal.target_position = target_position

        self.get_logger().info(f'Sending gripper goal: position={target_position:.4f}')

        self._gripper_client.wait_for_server()
        future = self._gripper_client.send_goal_async(
            goal, feedback_callback=self._gripper_feedback_callback)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'Gripper goal succeeded: {result.message}')
        else:
            self.get_logger().error(f'Gripper goal failed: {result.message}')
        return result.success

    def _pose_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Pose feedback: distance_to_goal={fb.distance_to_goal:.4f}')

    def _gripper_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'Gripper feedback: current_position={fb.current_position:.4f}')

    def run(self):
        """Load poses from CSV and execute the sequence."""
        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.get_logger().info(f'Loading poses from: {csv_path}')

        poses = load_poses(csv_path)
        self.get_logger().info(f'Loaded {len(poses)} pose(s)')

        self.get_logger().info('=== Starting go_to_poses sequence ===')

        for i, step in enumerate(SEQUENCE):
            self.get_logger().info(f'--- Step {i + 1}/{len(SEQUENCE)}: {step[0]} ---')

            if step[0] == 'pose':
                idx = step[1]
                if idx >= len(poses):
                    self.get_logger().error(
                        f'Pose index {idx} out of range (only {len(poses)} poses loaded)')
                    return
                success = self.send_pose_goal(poses[idx])

            elif step[0] == 'gripper':
                success = self.send_gripper_goal(step[1])

            else:
                self.get_logger().error(f'Unknown step type: {step[0]}')
                return

            if not success:
                self.get_logger().error(f'Step {i + 1} failed. Aborting.')
                return

        self.get_logger().info('=== go_to_poses sequence complete ===')


def main(args=None):
    rclpy.init(args=args)

    node = GoToPoses()
    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
