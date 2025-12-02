#! /usr/bin/env python3

# SPDX-FileCopyrightText: 2024 Marc Bestmann <marc.bestmann@dlr.de>
#
# SPDX-License-Identifier: MIT

"""This Module implements an interactive marker which is used to select datapoints."""
import sys

from geometry_msgs.msg import Point, PointStamped
from interactive_markers import InteractiveMarkerServer
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from vinspect_msgs.msg import Settings
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, \
    InteractiveMarkerFeedback, Marker


class SelectionMarker(Node):
    """Handles the interactive marker to select data points."""

    def __init__(self):
        """
        Initialize the interactive marker.

        :param node: ROS node
        :param inspection: Inspection object
        :return: None
        """
        super(SelectionMarker, self).__init__('selection_marker')
        self.sphere_radius = 0.2
        self.server = InteractiveMarkerServer(self, 'selection_marker')
        self.position = Point(x=0.15, y=0.05, z=0.1)
        self.create_sphere_marker(
            InteractiveMarkerControl.MOVE_3D, self.position)
        self.server.applyChanges()
        self.settings_sub = self.create_subscription(
            Settings, 'vinspect/settings', self.settings_cb, 10)
        self.clicked_point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_point_cb, 10)
        self.update_sphere_marker()

    def settings_cb(self, msg: Settings) -> None:
        if msg.sphere_radius != 0 and msg.sphere_radius != self.sphere_radius:
            self.sphere_radius = msg.sphere_radius
            print(f'{self.sphere_radius}')
            self.update_sphere_marker()

    def clicked_point_cb(self, msg: PointStamped) -> None:
        self.position = msg.point
        self.update_sphere_marker()

    def process_sphere_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        self.position = feedback.pose.position

    def create_sphere(self) -> Marker:
        """
        Is called to create a marker.sphere object.

        :return: Marker.SPHERE object
        """
        marker = Marker()

        marker.type = Marker.SPHERE
        marker.scale.x = self.sphere_radius * 2
        marker.scale.y = self.sphere_radius * 2
        marker.scale.z = self.sphere_radius * 2
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 0.4

        return marker

    def create_sphere_control(self) -> InteractiveMarkerControl:
        """
        Is called to set the Control object for the interactice marker.

        :return: control obejct
        """
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(self.create_sphere())
        return control

    def normalize_quaternion(self, quaternion_msg):
        """Todo DocString and why is this neccesarry? Copied from Interactive Marker tutotial."""
        norm = quaternion_msg.x**2 + quaternion_msg.y**2 + \
            quaternion_msg.z**2 + quaternion_msg.w**2
        s = norm ** (-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s

    def create_sphere_marker(self, interaction_mode: InteractiveMarkerControl, position: Point) \
            -> None:
        """
        Is called to create the interactive marker and to give this object to the interactive \
        marker server.

        :param interaction_mode: way of controling the object. Either MOVE_3D or MOVE_ROTATE_3D
        :param position: starting postion
        :return: None
        """
        int_marker = InteractiveMarker()
        # todo maybe make this a parameter
        int_marker.header.frame_id = 'world'
        int_marker.pose.position = position

        int_marker.name = 'simple_3dof'
        int_marker.description = 'Simple 3-DOF Control'

        # insert a sphere
        int_marker.controls.append(self.create_sphere_control())
        int_marker.controls[0].interaction_mode = interaction_mode
        int_marker.scale = int_marker.controls[0].markers[0].scale.x

        if interaction_mode != InteractiveMarkerControl.NONE:
            control_modes_dict = {
                InteractiveMarkerControl.MOVE_3D: 'MOVE_3D',
                InteractiveMarkerControl.MOVE_ROTATE_3D: 'MOVE_ROTATE_3D',
            }
            int_marker.name += '_' + control_modes_dict[interaction_mode]
            int_marker.description = '3D Control'
            int_marker.description += '\n' + \
                control_modes_dict[interaction_mode]

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        self.normalize_quaternion(control.orientation)
        control.name = 'move_x'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        self.normalize_quaternion(control.orientation)
        control.name = 'move_z'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        self.normalize_quaternion(control.orientation)
        control.name = 'move_y'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        self.server.insert(
            int_marker, feedback_callback=self.process_sphere_feedback)

        self.get_logger().info(f'Inserted marker: {int_marker.name}')

    def update_sphere_marker(self) -> None:
        self.server.erase('simple_3dof')
        self.create_sphere_marker(
            InteractiveMarkerControl.MOVE_3D, self.position)
        self.server.applyChanges()


def main(args=None):
    rclpy.init(args=None)
    node = SelectionMarker()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main(sys.argv)
