// Copyright 2026 Australian Robotics Supplies & Technology
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

#ifndef USB_MSGS__USB_TRANSPORT_ENCODINGS_HPP_
#define USB_MSGS__USB_TRANSPORT_ENCODINGS_HPP_

#include <array>
#include <string>
#include <string_view>

namespace usb_msgs
{
namespace usb_transport_encodings
{

// Canonical USBFrame transport encoding names.
//
// RAW:
//   The data field contains the original, uncompressed USB frame.
//
// GZIP:
//   DEFLATE-compressed data using gzip framing.
//
// ZLIB:
//   DEFLATE-compressed data using zlib framing.
//
// LZ4:
//   LZ4-compressed data. The exact framing should be defined by usb_transport.
//
// ZSTD:
//   Zstandard-compressed data.
inline constexpr std::string_view RAW{"raw"};
inline constexpr std::string_view GZIP{"gzip"};
inline constexpr std::string_view ZLIB{"zlib"};
inline constexpr std::string_view LZ4{"lz4"};
inline constexpr std::string_view ZSTD{"zstd"};

inline constexpr std::array<std::string_view, 5> ENCODINGS{
  RAW,
  GZIP,
  ZLIB,
  LZ4,
  ZSTD
};

[[nodiscard]]
inline constexpr bool isRawEncoding(const std::string_view encoding) noexcept
{
  return encoding == RAW;
}

[[nodiscard]]
inline constexpr bool isGzipEncoding(const std::string_view encoding) noexcept
{
  return encoding == GZIP;
}

[[nodiscard]]
inline constexpr bool isZlibEncoding(const std::string_view encoding) noexcept
{
  return encoding == ZLIB;
}

[[nodiscard]]
inline constexpr bool isLz4Encoding(const std::string_view encoding) noexcept
{
  return encoding == LZ4;
}

[[nodiscard]]
inline constexpr bool isZstdEncoding(const std::string_view encoding) noexcept
{
  return encoding == ZSTD;
}

[[nodiscard]]
inline constexpr bool isCompressedEncoding(const std::string_view encoding) noexcept
{
  return encoding == GZIP ||
         encoding == ZLIB ||
         encoding == LZ4 ||
         encoding == ZSTD;
}

[[nodiscard]]
inline constexpr bool isSupportedEncoding(const std::string_view encoding) noexcept
{
  for (const auto supported_encoding : ENCODINGS) {
    if (encoding == supported_encoding) {
      return true;
    }
  }

  return false;
}

// Convenience overloads for generated ROS message string fields.
[[nodiscard]]
inline bool isRawEncoding(const std::string & encoding) noexcept
{
  return isRawEncoding(std::string_view{encoding});
}

[[nodiscard]]
inline bool isGzipEncoding(const std::string & encoding) noexcept
{
  return isGzipEncoding(std::string_view{encoding});
}

[[nodiscard]]
inline bool isZlibEncoding(const std::string & encoding) noexcept
{
  return isZlibEncoding(std::string_view{encoding});
}

[[nodiscard]]
inline bool isLz4Encoding(const std::string & encoding) noexcept
{
  return isLz4Encoding(std::string_view{encoding});
}

[[nodiscard]]
inline bool isZstdEncoding(const std::string & encoding) noexcept
{
  return isZstdEncoding(std::string_view{encoding});
}

[[nodiscard]]
inline bool isCompressedEncoding(const std::string & encoding) noexcept
{
  return isCompressedEncoding(std::string_view{encoding});
}

[[nodiscard]]
inline bool isSupportedEncoding(const std::string & encoding) noexcept
{
  return isSupportedEncoding(std::string_view{encoding});
}

}  // namespace usb_transport_encodings
}  // namespace usb_msgs

#endif  // USB_MSGS__USB_TRANSPORT_ENCODINGS_HPP_
