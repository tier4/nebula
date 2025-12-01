// Copyright 2025 TIER IV, Inc.
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

#include "nebula_core_common/util/expected.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <utility>

using nebula::util::bad_expected_access;
using nebula::util::expected;

class TestError : public std::exception
{
  std::string message_;

public:
  explicit TestError(std::string msg) : message_(std::move(msg)) {}
  [[nodiscard]] const char * what() const noexcept override { return message_.c_str(); }
};

TEST(ExpectedTest, HasValueWhenContainingValue)
{
  expected<int, std::string> e = 42;
  EXPECT_TRUE(e.has_value());
}

TEST(ExpectedTest, HasValueFalseWhenContainingError)
{
  expected<int, std::string> e = std::string{"error"};
  EXPECT_FALSE(e.has_value());
}

TEST(ExpectedTest, ValueReturnsContainedValue)
{
  expected<int, std::string> e = 42;
  EXPECT_EQ(e.value(), 42);
}

TEST(ExpectedTest, ValueThrowsWhenContainingError)
{
  expected<int, std::string> e = std::string{"error message"};
  EXPECT_THROW(static_cast<void>(e.value()), bad_expected_access);
}

TEST(ExpectedTest, ErrorReturnsContainedError)
{
  expected<int, std::string> e = std::string{"my error"};
  EXPECT_EQ(e.error(), "my error");
}

TEST(ExpectedTest, ErrorThrowsWhenContainingValue)
{
  expected<int, std::string> e = 42;
  EXPECT_THROW(static_cast<void>(e.error()), bad_expected_access);
}

TEST(ExpectedTest, ValueOrReturnsValueWhenPresent)
{
  expected<int, std::string> e = 42;
  EXPECT_EQ(e.value_or(100), 42);
}

TEST(ExpectedTest, ValueOrReturnsDefaultWhenError)
{
  expected<int, std::string> e = std::string{"error"};
  EXPECT_EQ(e.value_or(100), 100);
}

TEST(ExpectedTest, ErrorOrReturnsErrorWhenPresent)
{
  expected<int, std::string> e = std::string{"my error"};
  EXPECT_EQ(e.error_or("default error"), "my error");
}

TEST(ExpectedTest, ErrorOrReturnsDefaultWhenValue)
{
  expected<int, std::string> e = 42;
  EXPECT_EQ(e.error_or("default error"), "default error");
}

TEST(ExpectedTest, ValueOrThrowWithMessageThrowsRuntimeError)
{
  expected<int, std::string> e = std::string{"internal error"};
  EXPECT_THROW(static_cast<void>(e.value_or_throw("custom error message")), std::runtime_error);

  try {
    e.value_or_throw("custom error message");
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error & ex) {
    EXPECT_STREQ(ex.what(), "custom error message");
  }
}

TEST(ExpectedTest, ValueOrThrowWithMessageReturnsValueWhenPresent)
{
  expected<int, std::string> e = 42;
  EXPECT_EQ(e.value_or_throw("should not throw"), 42);
}

TEST(ExpectedTest, ValueOrThrowThrowsStoredError)
{
  expected<int, TestError> e = TestError("stored error");
  EXPECT_THROW(static_cast<void>(e.value_or_throw()), TestError);

  try {
    e.value_or_throw();
    FAIL() << "Expected TestError";
  } catch (const TestError & ex) {
    EXPECT_STREQ(ex.what(), "stored error");
  }
}

TEST(ExpectedTest, ValueOrThrowReturnsValueWhenPresent)
{
  expected<int, TestError> e = 42;
  EXPECT_EQ(e.value_or_throw(), 42);
}

TEST(ExpectedTest, WorksWithStringValue)
{
  expected<std::string, int> e = std::string{"hello"};
  EXPECT_TRUE(e.has_value());
  EXPECT_EQ(e.value(), "hello");
}

TEST(ExpectedTest, WorksWithComplexTypes)
{
  struct ComplexValue
  {
    int a;
    std::string b;
  };

  expected<ComplexValue, std::string> e = ComplexValue{42, "test"};
  EXPECT_TRUE(e.has_value());
  EXPECT_EQ(e.value().a, 42);
  EXPECT_EQ(e.value().b, "test");
}

TEST(ExpectedTest, BadExpectedAccessHasMessage)
{
  bad_expected_access ex("test message");
  EXPECT_STREQ(ex.what(), "test message");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
