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

// # --8<-- [start:include]
#include "nebula_core_common/util/expected.hpp"
// # --8<-- [end:include]

#include <cstdint>
#include <string>
#include <variant>

struct Config
{
  std::string required_field;
  int percentage_field{0};
};

// # --8<-- [start:usage]
enum class ValidationError : uint8_t {
  MissingField,
  OutOfRange,
};

nebula::util::expected<std::monostate, ValidationError> validate(const Config & config)
{
  if (config.required_field.empty()) {
    return ValidationError::MissingField;
  }

  if (config.percentage_field < 0 || config.percentage_field > 100) {
    return ValidationError::OutOfRange;
  }

  return std::monostate{};
}
// # --8<-- [end:usage]
