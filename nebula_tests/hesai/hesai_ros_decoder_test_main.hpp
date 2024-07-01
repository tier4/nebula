#pragma once

#include "hesai_ros_decoder_test.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

namespace nebula
{
namespace test
{
class DecoderTest : public ::testing::TestWithParam<nebula::ros::HesaiRosDecoderTestParams>
{
protected:
  /// @brief Instantiates the Hesai driver node with the given test parameters
  void SetUp() override;

  /// @brief Destroys the Hesai driver node
  void TearDown() override;

  std::shared_ptr<nebula::ros::HesaiRosDecoderTest> hesai_driver_;
  std::shared_ptr<rclcpp::Logger> logger_;
};

}  // namespace test
}  // namespace nebula
