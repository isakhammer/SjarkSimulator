import os
import unittest
import xml.etree.ElementTree as ET

import launch
import launch_testing
import launch_ros.actions
import pytest
import rclpy
from rcl_interfaces.srv import GetParameters


def _wait_for_robot_description(node, timeout_sec):
    client = node.create_client(GetParameters, "/robot_state_publisher/get_parameters")
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.destroy_client(client)
        return None
    request = GetParameters.Request()
    request.names = ["robot_description"]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    try:
        result = future.result()
    finally:
        node.destroy_client(client)
    if result is None or not result.values:
        return None
    return result.values[0].string_value


def _read_robot_name():
    urdf_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "urdf",
            "boat.urdf",
        )
    )
    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()
    try:
        return ET.fromstring(robot_description).attrib.get("name", "")
    except ET.ParseError:
        return ""


@pytest.mark.launch_test
def generate_test_description():
    urdf_path = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "urdf",
            "boat.urdf",
        )
    )
    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    return (
        launch.LaunchDescription(
            [
                robot_state_publisher,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {},
    )


class TestUrdfPublisher(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("urdf_publisher_test_listener")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_robot_description_is_published(self):
        description = _wait_for_robot_description(self.node, timeout_sec=5.0)
        self.assertIsNotNone(description, "robot_description parameter not available")
        self.assertIn("<robot", description, "robot_description does not look like URDF")
        robot_name = _read_robot_name()
        if robot_name:
            self.assertIn(
                robot_name,
                description,
                "URDF does not include the expected robot name",
            )
