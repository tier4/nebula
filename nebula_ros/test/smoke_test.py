import time
import unittest

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy


def resolve_launch_file(context: LaunchContext, *args, **kwargs):
    sensor_model = LaunchConfiguration("sensor_model").perform(context)
    launch_file_path = LaunchConfiguration("launch_file_path").perform(context)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_path),
            launch_arguments=[("sensor_model", sensor_model), ("launch_hw", "false")],
        )
    ]


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription(
        [OpaqueFunction(function=resolve_launch_file), launch_testing.actions.ReadyToTest()]
    )


class DummyTest(unittest.TestCase):
    def test_wait_for_node_ready(self):
        """Waiting for the node coming online."""
        rclpy.init()
        test_node = rclpy.create_node("test_node")
        # Wait until both dummy node "test_node" and real tested node are registered and then kill
        # both of them, if tested node does not register within `timeout` seconds test will fail
        start_time = time.time()
        timeout = 2  # seconds
        timeout_msg = "Smoke test timeout has been exceeded ({}s)".format(timeout)
        print("waiting for nodes to be ready")
        while len(test_node.get_node_names()) < 2:
            assert time.time() - start_time < timeout, timeout_msg
            time.sleep(0.1)
        rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
