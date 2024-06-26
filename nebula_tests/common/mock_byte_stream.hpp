// Copyright 2024 TIER IV, Inc.
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

#include <nebula_hw_interfaces/nebula_hw_interfaces_common/connections/byte_stream.hpp>

#include <cstdint>
#include <vector>

namespace nebula::test
{

class MockByteStream final : public drivers::connections::PullableByteStream
{
public:
  explicit MockByteStream(const std::vector<std::vector<uint8_t>> & stream) : stream_(stream) {}

  void read(std::vector<uint8_t> & into, size_t /* n_bytes */) override
  {
    auto from = stream_[index_++];
    into.clear();
    into.insert(into.end(), from.cbegin(), from.cend());
  }

private:
  const std::vector<std::vector<uint8_t>> & stream_;
  size_t index_{};
};

}  // namespace nebula::test
