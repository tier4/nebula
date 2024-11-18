// Copyright 2024 TIER IV, Inc.

#include "aeva_hw_interface_test.hpp"

#include "../common/mock_byte_stream.hpp"
#include "../data/aeva/tcp_stream.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_aeva/connections/pointcloud.hpp"

#include <nebula_common/aeva/config_types.hpp>
#include <nebula_hw_interfaces/nebula_hw_interfaces_aeva/connections/aeva_api.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <thread>

using nebula::drivers::aeva::PointCloudMessage;
using nebula::drivers::connections::aeva::PointcloudParser;

TEST(TestParsing, Pointcloud)  // NOLINT
{
  using std::chrono_literals::operator""ms;

  auto mock_byte_stream = std::make_shared<nebula::test::MockByteStream>(STREAM);
  PointcloudParser parser(mock_byte_stream);
  std::atomic_bool done = false;

  PointcloudParser::callback_t callback = [&](const PointCloudMessage & arg) {
    if (done) return;

    EXPECT_EQ(arg.header.aeva_marker, 0xAE5Au);
    EXPECT_EQ(arg.header.platform, 2);

    EXPECT_GT(arg.points.size(), 0);
    EXPECT_TRUE(mock_byte_stream->done());
    EXPECT_EQ(mock_byte_stream->get_read_count(), 2);
    done = true;
  };

  parser.register_callback(std::move(callback));

  mock_byte_stream->run();
  while (!done) {
    std::this_thread::yield();
  }
}
