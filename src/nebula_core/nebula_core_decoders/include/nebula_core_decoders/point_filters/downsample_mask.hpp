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

#include "nebula_core_decoders/angles.hpp"

#include <Eigen/Core>
#include <nebula_core_common/loggers/logger.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/point_types.hpp>
#include <nebula_core_common/util/string_conversions.hpp>

#include <png.h>
#include <sys/types.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace nebula::drivers::point_filters
{

/** A function that takes an x (azimuth) and y (channel) coordinate and outputs
 * a combined dither pattern index. The default implementation is (x + y) and yields
 * a dither pattern skewed by 45deg.
 *
 * Some scan patterns (e.g. OT128 hi-res mode) have their own dithering patterns that
 * might interfere with this default. Providing a custom function with knowledge of
 * such interferences can improve issues like flickering or banding.
 */
using DitherTransform = std::function<size_t(size_t x, size_t y)>;
const DitherTransform g_default_dither_transform = [](size_t x, size_t y) { return x + y; };

namespace impl
{

/// An 8-bit grayscale image, stored row by row.
class GrayImage
{
public:
  GrayImage(size_t width, size_t height) : width_(width), height_(height), data_(width * height) {}

  /**
   * @brief Load a PNG file, converting it to 8-bit grayscale if it is in another format.
   *
   * @throws std::runtime_error if the file cannot be read or is not a valid PNG.
   */
  static GrayImage read(const std::filesystem::path & path)
  {
    png_image png{};
    png.version = PNG_IMAGE_VERSION;

    if (png_image_begin_read_from_file(&png, path.c_str()) == 0) {
      throw std::runtime_error("Could not read " + path.native() + ": " + png.message);
    }

    png.format = PNG_FORMAT_GRAY;
    GrayImage image(png.width, png.height);

    if (png_image_finish_read(&png, nullptr, image.data_.data(), 0, nullptr) == 0) {
      std::string message = png.message;
      png_image_free(&png);
      throw std::runtime_error("Could not read " + path.native() + ": " + message);
    }

    return image;
  }

  /**
   * @brief Write the image to a PNG file.
   *
   * @throws std::runtime_error if the file cannot be written.
   */
  void write(const std::filesystem::path & path) const
  {
    png_image png{};
    png.version = PNG_IMAGE_VERSION;
    png.width = static_cast<png_uint_32>(width_);
    png.height = static_cast<png_uint_32>(height_);
    png.format = PNG_FORMAT_GRAY;

    if (png_image_write_to_file(&png, path.c_str(), 0, data_.data(), 0, nullptr) == 0) {
      throw std::runtime_error("Could not write " + path.native() + ": " + png.message);
    }
  }

  [[nodiscard]] size_t get_width() const { return width_; }
  [[nodiscard]] size_t get_height() const { return height_; }

  [[nodiscard]] uint8_t get_pixel(size_t x, size_t y) const { return data_[y * width_ + x]; }
  void set_pixel(size_t x, size_t y, uint8_t value) { data_[y * width_ + x] = value; }

private:
  size_t width_;
  size_t height_;
  std::vector<uint8_t> data_;
};

inline void dither(
  const GrayImage & in, GrayImage & out, uint8_t quantization_levels,
  const DitherTransform & transform)
{
  if (in.get_width() != out.get_width() || in.get_height() != out.get_height()) {
    std::stringstream ss;
    ss << "Expected downsample mask of size "
       << "(" << out.get_width() << ", " << out.get_height() << ")";
    ss << ", got "
       << "(" << in.get_width() << ", " << in.get_height() << ")";

    throw std::runtime_error(ss.str());
  }

  uint32_t denominator = quantization_levels;

  auto should_keep = [denominator](uint32_t numerator, uint32_t pos) {
    for (uint32_t i = 0; i < numerator; ++i) {
      auto dithered_pos =
        static_cast<size_t>(std::round(denominator / static_cast<double>(numerator) * i));
      if (dithered_pos == pos) return true;
    }
    return false;
  };

  for (size_t y = 0; y < out.get_height(); ++y) {
    for (size_t x = 0; x < out.get_width(); ++x) {
      uint32_t numerator = static_cast<uint32_t>(in.get_pixel(x, y)) * denominator / 255;
      size_t pos = transform(x, y) % denominator;
      bool keep = should_keep(numerator, pos);
      out.set_pixel(x, y, keep * 255);
    }
  }
}

}  // namespace impl

class DownsampleMaskFilter
{
  static const uint8_t g_quantization_levels = 10;

public:
  DownsampleMaskFilter(
    const std::string & filename, AngleRange<int32_t, MilliDegrees> azimuth_range_mdeg,
    uint32_t azimuth_peak_resolution_mdeg, size_t n_channels,
    const std::shared_ptr<loggers::Logger> & logger, bool export_dithered_mask,
    const DitherTransform & dither_transform)
  : azimuth_range_{
      deg2rad(azimuth_range_mdeg.start / 1000.), deg2rad(azimuth_range_mdeg.end / 1000.)}
  {
    if (azimuth_peak_resolution_mdeg == 0) {
      throw std::invalid_argument("azimuth_peak_resolution_mdeg must be positive");
    }
    if (azimuth_range_.extent() <= 0) {
      throw std::invalid_argument("azimuth range extent must be positive");
    }
    if (n_channels == 0) {
      throw std::invalid_argument("n_channels must be positive");
    }

    auto factors = impl::GrayImage::read(filename);

    size_t mask_cols = azimuth_range_mdeg.extent() / azimuth_peak_resolution_mdeg;
    size_t mask_rows = n_channels;

    impl::GrayImage dithered(mask_cols, mask_rows);
    impl::dither(factors, dithered, g_quantization_levels, dither_transform);

    mask_ = Eigen::MatrixX<uint8_t>(mask_rows, mask_cols);

    for (size_t y = 0; y < dithered.get_height(); ++y) {
      for (size_t x = 0; x < dithered.get_width(); ++x) {
        mask_.coeffRef(static_cast<int32_t>(y), static_cast<int32_t>(x)) = dithered.get_pixel(x, y);
      }
    }

    if (export_dithered_mask) {
      std::filesystem::path out_path{filename};
      out_path = out_path.replace_filename(
        out_path.stem().string() + "_dithered" + out_path.extension().string());

      try {
        dithered.write(out_path);
        logger->info("Wrote dithered mask to " + out_path.native());
      } catch (const std::runtime_error & e) {
        logger->warn(e.what());
      }
    }
  }

  [[nodiscard]] bool excluded(const NebulaPoint & point) const
  {
    double azi_normalized = (point.azimuth - azimuth_range_.start) / azimuth_range_.extent();

    auto x = static_cast<ssize_t>(std::round(azi_normalized * static_cast<double>(mask_.cols())));
    auto y = point.channel;

    bool x_out_of_bounds = x < 0 || x >= mask_.cols();
    bool y_out_of_bounds = y >= mask_.rows();

    return x_out_of_bounds || y_out_of_bounds || !mask_.coeff(y, x);
  }

private:
  AngleRange<double, Radians> azimuth_range_;
  Eigen::MatrixX<uint8_t> mask_;
};

}  // namespace nebula::drivers::point_filters
