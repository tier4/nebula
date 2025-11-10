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
        [
            OpaqueFunction(function=resolve_launch_file),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestCorrectStartup(unittest.TestCase):
    def test_wait_for_startup_then_shutdown(self):
        self.proc_output: launch_testing.ActiveIoHandler
        self.proc_output.assertWaitFor("Hardware connection disabled", timeout=2)


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
