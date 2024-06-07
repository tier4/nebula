import unittest

import launch
import launch_ros
import launch_testing
import launch_testing.event_handlers
import pytest


@pytest.mark.rostest
def generate_test_description():
    test_node = launch_ros.actions.Node(
        package="nebula_tests",
        executable="hesai_ros_decoder_test_node_standalone",
        parameters=["test/hesai_ros_decoder_test.yaml"],
    )
    return (
        launch.LaunchDescription(
            [
                # other fixture actions
                test_node,
                launch_testing.util.KeepAliveProc(),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        locals(),
    )


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0])
