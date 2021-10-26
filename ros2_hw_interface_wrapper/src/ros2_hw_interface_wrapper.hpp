#ifndef NEBULA_DRIVER_ROS2_HW_INTERFACE_WRAPPER_HPP
#define NEBULA_DRIVER_ROS2_HW_INTERFACE_WRAPPER_HPP
/// @class RosDriverWrapper
namespace lidar_driver
{
class RosDriverWrapper final : public rclcpp::Node {
  public:
    explicit RosHwInterfaceWrapper(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
        const std::string & node_name = "lidar_hw_interface_node");
    ~RosHwInterfaceWrapper() override;

    RosHwInterfaceWrapper(RosHwInterfaceWrapper && c) = delete;
    RosHwInterfaceWrapper & operator=(RosHwInterfaceWrapper && c) = delete;
    RosHwInterfaceWrapper(const RosHwInterfaceWrapper & c) = delete;
    RosHwInterfaceWrapper & operator=(const RosHwInterfaceWrapper & c) = delete;

  private:
    void MainThread();
    std::thread main_thread_;
    std::shared_future<void> future_;
    std::promise<void> exit_signal_;

  std::string calibration_configuration_file_;
  CalibrationConfiguration calibration_configuration_;
  // Load the calibration file if it exits, otherwise save to file
  /// Define default file locations, probably in the individual drivers
  bool LoadCalibrationConfiguration(std::string file_name);
  void GetCalibrationConfiguration();


#endif //NEBULA_DRIVER_ROS2_HW_INTERFACE_WRAPPER_HPP
