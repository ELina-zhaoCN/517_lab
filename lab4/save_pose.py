#!/usr/bin/env python3
"""Save end-effector pose service node.

Provides the /follower/save_pose service (soa_interfaces/srv/SavePose)
to capture the current pose of the gripper_link in the base_link frame
using tf2_ros, and optionally append it to a CSV file.

Usage:
    ros2 run soa_functions save_pose

Services:
    /follower/save_pose (soa_interfaces/srv/SavePose)
        request:  csv_path — path to CSV file; if empty, pose is returned but not saved
        response: success, pose
"""

import csv
import os

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import Pose
from soa_interfaces.srv import SavePose


class SavePoseNode(Node):

    def __init__(self):
        super().__init__('save_pose')

        self._cb_group = ReentrantCallbackGroup()

        # tf2 buffer and listener to look up transforms
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_service(
            SavePose,
            '/follower/save_pose',
            self._handle_save_pose,
            callback_group=self._cb_group,
        )

        self.get_logger().info('SavePose service ready.')

    def _handle_save_pose(self, req, res):
        try:
            # Look up transform: gripper_link relative to base_link
            transform = self._tf_buffer.lookup_transform(
                'base_link',
                'gripper_link',
                rclpy.time.Time(),
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            res.success = False
            return res

        t = transform.transform.translation
        r = transform.transform.rotation

        pose = Pose()
        pose.position.x = t.x
        pose.position.y = t.y
        pose.position.z = t.z
        pose.orientation.x = r.x
        pose.orientation.y = r.y
        pose.orientation.z = r.z
        pose.orientation.w = r.w

        res.pose = pose
        res.success = True

        if req.csv_path:
            try:
                self._append_to_csv(req.csv_path, pose)
            except OSError as e:
                self.get_logger().error(f'Failed to write CSV: {e}')
                res.success = False

        return res

    def _append_to_csv(self, path: str, pose: Pose) -> None:
        fieldnames = ['pos_x', 'pos_y', 'pos_z',
                      'ori_x', 'ori_y', 'ori_z', 'ori_w']
        file_exists = os.path.isfile(path)
        with open(path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()
            writer.writerow({
                'pos_x': pose.position.x,
                'pos_y': pose.position.y,
                'pos_z': pose.position.z,
                'ori_x': pose.orientation.x,
                'ori_y': pose.orientation.y,
                'ori_z': pose.orientation.z,
                'ori_w': pose.orientation.w,
            })
        self.get_logger().info(f'Saved pose to {path}')


def main(args=None):
    rclpy.init(args=args)
    node = SavePoseNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
