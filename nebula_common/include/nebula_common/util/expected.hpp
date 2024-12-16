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

#pragma once

#include <exception>
#include <stdexcept>
#include <string>
#include <variant>

namespace nebula
{
namespace util
{

struct bad_expected_access : public std::exception
{
  explicit bad_expected_access(const std::string & msg) : std::exception(), msg_(msg) {}

  const char * what() const noexcept override { return msg_.c_str(); }

private:
  const std::string msg_;
};

/// @brief A poor man's backport of C++23's std::expected.
///
/// At any given time, holds exactly one of: expected value, or error value.
/// Provides functions for easy exception handling.
///
/// More info here: https://en.cppreference.com/w/cpp/utility/expected
///
/// @tparam T Type of the expected value
/// @tparam E Error type
template <typename T, typename E>
struct expected
{
  /// @brief Whether the expected instance holds a value (as opposed to an error).
  /// Call this before trying to access values via `value()`.
  /// @return True if a value is contained, false if an error is contained
  bool has_value() { return std::holds_alternative<T>(value_); }

  /// @brief Retrieve the value, or throw `bad_expected_access` if an error is contained.
  /// @return The value of type `T`
  T value()
  {
    if (!has_value()) {
      throw bad_expected_access("value() called but containing error");
    }
    return std::get<T>(value_);
  }

  /// @brief Return the contained value, or, if an error is contained, return the given `default_`
  /// instead.
  /// @return The contained value, if any, else `default_`
  T value_or(const T & default_)
  {
    if (has_value()) return value();
    return default_;
  }

  /// @brief If the instance has a value, return the value, else throw std::runtime_error(error_msg)
  /// @param error_msg The message to be included in the thrown exception
  /// @return The value
  T value_or_throw(const std::string & error_msg)
  {
    if (has_value()) return value();
    throw std::runtime_error(error_msg);
  }

  /// @brief If the instance has a value, return the value, else throw the stored error instance.
  T value_or_throw()
  {
    if (has_value()) return value();
    throw error();
  }

  /// @brief Retrieve the error, or throw `bad_expected_access` if a value is contained.
  /// @return The error of type `E`
  E error()
  {
    if (has_value()) {
      throw bad_expected_access("error() called but containing value");
    }
    return std::get<E>(value_);
  }

  /// @brief Return the contained error, or, if a value is contained, return the given `default_`
  /// instead.
  /// @return The contained error, if any, else `default_`
  E error_or(const E & default_)
  {
    if (!has_value()) return error();
    return default_;
  }

  expected(const T & value) { value_ = value; }  // NOLINT(runtime/explicit)

  expected(const E & error) { value_ = error; }  // NOLINT(runtime/explicit)

private:
  std::variant<T, E> value_;
};

}  // namespace util
}  // namespace nebula
