#pragma once

#include <variant>

namespace nebula
{
namespace util
{

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
  bool has_value() { return std::holds_alternative<T>(value_); }

  T value() { return std::get<T>(value_); }

  T value_or(const T & default_)
  {
    if (has_value()) return value();
    return default_;
  }

  T value_or_throw(const std::string & error_msg) {
    if (has_value()) return value();
    throw std::runtime_error(error_msg);
  }

  E error() { return std::get<E>(value_); }

  E error_or(const E & default_) { 
    if (!has_value()) return error();
    return default_; 
    }

  expected(const T & value) { value_ = value; }

  expected(const E & error) { value_ = error; }

private:
  std::variant<T, E> value_;
};

}  // namespace util
}  // namespace nebula