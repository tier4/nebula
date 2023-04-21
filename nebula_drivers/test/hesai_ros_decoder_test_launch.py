import unittest

import launch
import launch_ros
import launch_testing
import launch_testing.event_handlers
import pytest


@pytest.mark.rostest
def generate_test_description():
    test_node = launch_ros.actions.Node(
        package='nebula_lidar_driver',
#        executable='hesai_ros_decoder_test_node',
        executable='hesai_ros_decoder_test_node_standalone',
        parameters=['test/hesai_ros_decoder_test.yaml'],
    )
    '''
    return launch.LaunchDescription([
        launch.actions.RegisterEventHandler(
            launch_testing.event_handlers.StdoutReadyListener(
                target_action=test_node,
                ready_txt='Spoofer(s) number is:',
                actions=[
                    launch_testing.util.KeepAliveProc(),
                    launch_testing.actions.ReadyToTest()
                    ]
            )
        ),


        # other fixture actions
#        test_node,
#        launch_testing.util.KeepAliveProc(),
#        launch_testing.actions.ReadyToTest()
    ]), locals()
    '''
    return launch.LaunchDescription([
        # other fixture actions
        test_node,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest()
    ]), locals()

@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):

#    def test_error_message(self, proc_output, put):
#        launch_testing.asserts.assertInStdout(proc_output, 'foobar', put)

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0])

# https://github.com/GitRepJo/ros2_cpp_test_example

'''
@pytest.mark.launch_test
def generate_test_description():
    """ Specifiy nodes or processes to launch for test
        :param
        :return dut [ros2 node] node to be tested (device under test)
        :return ...,... specifications for launch_testing
    Multiple nodes that are to be tested can be launched"""

    # dut -> device under test is the node to be tested in this example
    dut = Node(
        package='nebula_lidar_driver',
        executable='sub_pub',
        name='sub_pub',
    )
    context = {'dut': dut  }

    return (launch.LaunchDescription([
        dut,
        launch_testing.actions.ReadyToTest()]
        ) , context
    )

class TestProcessOutput(unittest.TestCase):
    """Details to use this class in the context of launch_testing:
        nodes: https://github.com/ros2/launch_ros
        process: https://github.com/ros2/launch/tree/master/launch_testing"""

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('input_output_node')

    def tearDown(self):
        self.node.destroy_node()


    def timer_callback(self):
        """ Reads a file and publish the data from this file to ros2
                :param -
                :return -
            """
        # Read input data that is send to dut
        INPUT_DATA_PATH = os.path.join(
        ament_index_python.get_package_prefix('nebula_lidar_driver'),
        'lib/hesai_ros_decoder_test',
        'hesai_ros_decoder_test_param.yaml'
        )

        with open(INPUT_DATA_PATH) as f:
            data = yaml.safe_load(f)

        msg = String()
        msg.data = data['msg']['data']
        self.publisher_.publish(msg)
        self.node.get_logger().info('Publishing: "%s"' % msg.data)


    def test_dut_output(self, dut, proc_output):
        """ Listen for a message published by dut and compare message to expected value
                :param
                :return dut [ros2 node] node to be tested (device under test)
                :return proc_output [ActiveIoHandler] data output of dut as shown in terminal (stdout)
                :return -
            """
        # Get current functionname
        frame = inspect.currentframe()
        function_name = inspect.getframeinfo(frame).function

        # Publish data to dut
        self.publisher_ = self.node.create_publisher(String, 'input_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.node.create_timer(timer_period, self.timer_callback)

        # Read data of expected result
        EXPECTED_DATA_PATH = os.path.join(
        ament_index_python.get_package_prefix('ros2_cpp_test_example'),
        'lib/ros2_cpp_test_example',
        'expected_data.yaml'
        )

        with open(EXPECTED_DATA_PATH) as f:
            data = yaml.safe_load(f)
        expected_data = data['msg']['data']

        # Setup for listening to dut messages
        received_data =  []
        sub = self.node.create_subscription(
            String,
            'output_topic',
            lambda msg: received_data.append(str(msg.data)),
            10
        )

        try:
            # Wait until the dut transmits a message over the ROS topic
            end_time = time.time() + 1
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            if received_data == []:
                test_data = ""

            else:

                print (f"\n[{function_name}] [INFO] expected_data:\n"+ str(expected_data))
                print (f"\n[{function_name}] [INFO] received_data:\n"+ str(received_data[0]))
                test_data = received_data[0]

            # test actual output for expected output
            self.assertEqual(str(test_data),expected_data)

        finally:
            self.node.destroy_subscription(sub)
'''
