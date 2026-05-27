// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <nebula_core_runtime/live_transport_graph.hpp>
#include <nebula_core_runtime/sensor_registry.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>

namespace nebula::drivers::test
{

TEST(TestLiveTransportGraph, HttpGetThrowsForUnknownEndpoint)
{
  auto registry = std::make_shared<SensorRegistry>();
  LiveTransportGraph graph(registry);
  EXPECT_THROW(graph.http_get("nonexistent"), std::invalid_argument);
}

TEST(TestLiveTransportGraph, HttpPostThrowsForUnknownEndpoint)
{
  auto registry = std::make_shared<SensorRegistry>();
  LiveTransportGraph graph(registry);
  EXPECT_THROW(graph.http_post("nonexistent", "body"), std::invalid_argument);
}

TEST(TestLiveTransportGraph, StartStopOnUnconfiguredGraphIsNoop)
{
  auto registry = std::make_shared<SensorRegistry>();
  LiveTransportGraph graph(registry);
  EXPECT_NO_THROW(graph.start());
  EXPECT_NO_THROW(graph.stop());
}

TEST(TestLiveTransportGraph, SetCallbacksDoNotCrash)
{
  auto registry = std::make_shared<SensorRegistry>();
  LiveTransportGraph graph(registry);
  EXPECT_NO_THROW(graph.set_output_callback([](const SensorDecodedOutput &) {}));
  EXPECT_NO_THROW(graph.set_error_callback([](const SensorError &) {}));
  EXPECT_NO_THROW(graph.set_progress_callback([](const SensorProgress &) {}));
}

TEST(TestLiveTransportGraph, ConfigureThrowsForUnknownModel)
{
  auto registry = std::make_shared<SensorRegistry>();
  LiveTransportGraph graph(registry);
  LiveSessionConfig config;
  config.model = SensorModel::VELODYNE_VLP16;
  EXPECT_THROW(graph.configure(config), std::runtime_error);
}

}  // namespace nebula::drivers::test
