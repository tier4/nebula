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

#include "nebula_core_decoders/angles.hpp"
#include "nebula_core_decoders/point_filters/downsample_mask.hpp"

#include <nebula_core_common/loggers/console_logger.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/point_types.hpp>

#include <gtest/gtest.h>
#include <png.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef _TEST_RESOURCES_PATH
static_assert(false, " _TEST_RESOURCES_PATH is not defined");
#endif

static std::filesystem::path downsample_mask_path()
{
  return std::filesystem::path(_TEST_RESOURCES_PATH) / "downsample_masks";
}

TEST(TestDownsampleMask, TestDithering)
{
  using nebula::drivers::point_filters::g_default_dither_transform;
  using nebula::drivers::point_filters::impl::dither;
  using nebula::drivers::point_filters::impl::GrayscaleImage;

  size_t quantization_levels = 10;
  size_t w = 10;
  size_t h = 10;

  std::vector<uint32_t> keep_quantiles = {0, 33, 50, 67, 100};

  for (uint32_t quantile : keep_quantiles) {
    auto image_path = downsample_mask_path() / ("q" + std::to_string(quantile) + ".png");
    ASSERT_TRUE(std::filesystem::is_regular_file(image_path));

    auto in = GrayscaleImage::read(image_path);
    GrayscaleImage out{in.get_width(), in.get_height()};
    dither(in, out, quantization_levels, g_default_dither_transform);

    size_t n_kept = 0;
    for (size_t x = 0; x < w; ++x) {
      for (size_t y = 0; y < h; ++y) {
        uint8_t px = out.get_pixel(x, y);
        EXPECT_TRUE(px == 0 || px == 255);
        n_kept += out.get_pixel(x, y) / 255;
      }
    }

    EXPECT_NEAR(n_kept, quantile, 9);
    EXPECT_LE(n_kept, quantile);

    if (quantile % quantization_levels == 0) {
      EXPECT_EQ(n_kept, quantile);
    }
  }
}

TEST(TestDownsampleMask, TestFilter)
{
  using nebula::drivers::AngleRange;
  using nebula::drivers::MilliDegrees;
  using nebula::drivers::NebulaPoint;
  using nebula::drivers::loggers::ConsoleLogger;
  using nebula::drivers::point_filters::DownsampleMaskFilter;
  using nebula::drivers::point_filters::g_default_dither_transform;

  auto logger = std::make_shared<ConsoleLogger>("TestFilter");
  auto image_path = downsample_mask_path() / ("q50.png");
  AngleRange<int32_t, MilliDegrees> azi_range_mdeg{-5, 5};
  uint16_t azi_step_mdeg = 1;
  uint8_t n_channels = 10;

  DownsampleMaskFilter f{
    image_path.native(),       azi_range_mdeg, azi_step_mdeg, n_channels, logger, false,
    g_default_dither_transform};

  size_t n_kept = 0;
  int32_t azi_kept_min = std::numeric_limits<int32_t>::max();
  int32_t azi_kept_max = std::numeric_limits<int32_t>::min();
  uint8_t channel_kept_min = std::numeric_limits<uint8_t>::max();
  uint8_t channel_kept_max = std::numeric_limits<uint8_t>::min();

  // Test points across the whole range, and also slightly outside the bounds (bound +-1)
  for (int32_t azimuth_mdeg = azi_range_mdeg.start - 1; azimuth_mdeg < azi_range_mdeg.end + 1;
       azimuth_mdeg += azi_step_mdeg) {
    for (uint8_t channel = 0; channel < n_channels + 1; ++channel) {
      NebulaPoint p{};
      p.channel = channel;
      p.azimuth = nebula::drivers::deg2rad(azimuth_mdeg / 1000.);

      if (!f.excluded(p)) {
        n_kept++;
        azi_kept_min = std::min(azi_kept_min, azimuth_mdeg);
        azi_kept_max = std::max(azi_kept_max, azimuth_mdeg);
        channel_kept_min = std::min(channel_kept_min, channel);
        channel_kept_max = std::max(channel_kept_max, channel);
      }
    }
  }

  // Exactly one point for each mask pixel has been checked, number of points kept shall be equal to
  // the mask's number of white pixels
  EXPECT_EQ(n_kept, 50);

  // The mask has white pixels at all borders, so it is expected that points at all borders have
  // been kept. It is also expected that no points beyond the borders have been kept. The upper
  // bounds are exclusive, thus `max - 1` here
  EXPECT_EQ(azi_kept_min, azi_range_mdeg.start);
  EXPECT_EQ(azi_kept_max, azi_range_mdeg.end - 1);
  EXPECT_EQ(channel_kept_min, 0);
  EXPECT_EQ(channel_kept_max, n_channels - 1);
}

TEST(TestDownsampleMask, TestImageRoundTrip)
{
  using nebula::drivers::point_filters::impl::GrayscaleImage;

  auto path = std::filesystem::temp_directory_path() / "nebula_test_downsample_mask_round_trip.png";

  GrayscaleImage out{3, 2};
  for (size_t y = 0; y < out.get_height(); ++y) {
    for (size_t x = 0; x < out.get_width(); ++x) {
      out.set_pixel(x, y, static_cast<uint8_t>(50 * (y * out.get_width() + x)));
    }
  }
  out.write(path);

  auto in = GrayscaleImage::read(path);
  std::filesystem::remove(path);

  ASSERT_EQ(in.get_width(), out.get_width());
  ASSERT_EQ(in.get_height(), out.get_height());
  for (size_t y = 0; y < out.get_height(); ++y) {
    for (size_t x = 0; x < out.get_width(); ++x) {
      EXPECT_EQ(in.get_pixel(x, y), out.get_pixel(x, y));
    }
  }
}

TEST(TestDownsampleMask, TestImageReadFailure)
{
  using nebula::drivers::point_filters::impl::GrayscaleImage;

  EXPECT_THROW(
    GrayscaleImage::read(downsample_mask_path() / "does_not_exist.png"), std::runtime_error);
  EXPECT_THROW(GrayscaleImage::read(__FILE__), std::runtime_error);
}

TEST(TestDownsampleMask, TestImageReadRgb)
{
  using nebula::drivers::point_filters::impl::GrayscaleImage;

  auto path = std::filesystem::temp_directory_path() / "nebula_test_downsample_mask_rgb.png";

  // One row of RGB pixels: four neutral ones, then pure red, green and blue
  const std::vector<uint8_t> rgb = {0,  0,   0, 255, 255, 255, 128, 128, 128, 37, 37,
                                    37, 255, 0, 0,   0,   255, 0,   0,   0,   255};
  const size_t width = rgb.size() / 3;

  png_image png{};
  png.version = PNG_IMAGE_VERSION;
  png.width = static_cast<png_uint_32>(width);
  png.height = 1;
  png.format = PNG_FORMAT_RGB;
  ASSERT_NE(png_image_write_to_file(&png, path.c_str(), 0, rgb.data(), 0, nullptr), 0);

  auto in = GrayscaleImage::read(path);
  std::filesystem::remove(path);

  ASSERT_EQ(in.get_width(), width);
  ASSERT_EQ(in.get_height(), 1);

  // Neutral pixels keep their value, so a grayscale mask saved as RGB behaves like a grayscale one
  EXPECT_EQ(in.get_pixel(0, 0), 0);
  EXPECT_EQ(in.get_pixel(1, 0), 255);
  EXPECT_EQ(in.get_pixel(2, 0), 128);
  EXPECT_EQ(in.get_pixel(3, 0), 37);

  // Colors are reduced to their perceived brightness (Rec. 709 luminance, computed in linear
  // light): green counts the most, blue the least, and they do not simply average to 85
  EXPECT_NEAR(in.get_pixel(4, 0), 127, 2);
  EXPECT_NEAR(in.get_pixel(5, 0), 220, 2);
  EXPECT_NEAR(in.get_pixel(6, 0), 76, 2);
}

TEST(TestDownsampleMask, TestImageReadOversized)
{
  using nebula::drivers::point_filters::impl::GrayscaleImage;

  // A PNG whose header claims 2147483647 x 2147483647 pixels, followed by an empty IDAT chunk
  const std::vector<uint8_t> png = {
    0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d, 0x49, 0x48, 0x44,
    0x52, 0x7f, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0x08, 0x00, 0x00, 0x00, 0x00, 0x31,
    0xa2, 0x54, 0xba, 0x00, 0x00, 0x00, 0x00, 0x49, 0x44, 0x41, 0x54, 0x35, 0xaf, 0x06, 0x1e};

  auto path = std::filesystem::temp_directory_path() / "nebula_test_downsample_mask_oversized.png";
  {
    std::ofstream file{path, std::ios::binary};
    for (uint8_t byte : png) {
      file.put(static_cast<char>(byte));
    }
  }

  EXPECT_THROW(GrayscaleImage::read(path), std::exception);
  std::filesystem::remove(path);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  return result;
}
