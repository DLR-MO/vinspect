#! /usr/bin/env python3

# SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
#
# SPDX-License-Identifier: MIT

from argparse import ArgumentParser

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from vinspect.inspection import load
from vinspect_msgs.msg import Sparse

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument('-i', '--input-data-path',
                        help='Path to the input file containing data in vinspect format.')
    args = parser.parse_args()

    rclpy.init(args=None)
    node = Node('load_and_publish')
    publisher = node.create_publisher(Sparse, 'sparse', 10)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    inspection = load(args.input_data_path)
    i = 0
    while rclpy.ok():
        msg = Sparse()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = inspection.sparse_position[i][0]
        msg.pose.position.y = inspection.sparse_position[i][1]
        msg.pose.position.z = inspection.sparse_position[i][2]
        msg.pose.orientation.x = inspection.sparse_orientation[i][0]
        msg.pose.orientation.y = inspection.sparse_orientation[i][1]
        msg.pose.orientation.z = inspection.sparse_orientation[i][2]
        msg.pose.orientation.w = inspection.sparse_orientation[i][3]
        msg.data = inspection.sparse_value[i]
        i += 1
        i = i % inspection.sparse_data_count
        publisher.publish(msg)
        node.get_clock().sleep_for(Duration(seconds=0.01))
    rclpy.shutdown()
