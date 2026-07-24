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

#ifndef USB_MSGS__USB_APPLICATION_ENCODINGS_HPP_
#define USB_MSGS__USB_APPLICATION_ENCODINGS_HPP_

#include <array>
#include <string>
#include <string_view>

namespace usb_msgs
{
namespace usb_application_encodings
{

// The application protocol is not known.
inline constexpr std::string_view UNKNOWN{"unknown"};

// The data is intentionally treated as uninterpreted binary content.
inline constexpr std::string_view OPAQUE{"opaque"};

// Text and serial-style encodings.
inline constexpr std::string_view ASCII{"ascii"};
inline constexpr std::string_view UTF8{"utf-8"};

// GNSS protocols.
inline constexpr std::string_view UBX{"ubx"};
inline constexpr std::string_view RTCM3{"rtcm3"};
inline constexpr std::string_view NMEA0183{"nmea0183"};

// Common structured data encodings.
inline constexpr std::string_view JSON{"json"};
inline constexpr std::string_view CBOR{"cbor"};
inline constexpr std::string_view PROTOBUF{"protobuf"};

// Common media encodings.
inline constexpr std::string_view JPEG{"jpeg"};
inline constexpr std::string_view PNG{"png"};
inline constexpr std::string_view H264{"h264"};
inline constexpr std::string_view H265{"h265"};

inline constexpr std::array<std::string_view, 14> ENCODINGS{
  UNKNOWN,
  OPAQUE,
  ASCII,
  UTF8,
  UBX,
  RTCM3,
  NMEA0183,
  JSON,
  CBOR,
  PROTOBUF,
  JPEG,
  PNG,
  H264,
  H265
};

[[nodiscard]]
inline constexpr bool isSupportedEncoding(
  const std::string_view encoding) noexcept
{
  for (const auto supported_encoding : ENCODINGS) {
    if (encoding == supported_encoding) {
      return true;
    }
  }

  return false;
}

[[nodiscard]]
inline constexpr bool isGnssEncoding(
  const std::string_view encoding) noexcept
{
  return encoding == UBX ||
         encoding == RTCM3 ||
         encoding == NMEA0183;
}

[[nodiscard]]
inline constexpr bool isTextEncoding(
  const std::string_view encoding) noexcept
{
  return encoding == ASCII ||
         encoding == UTF8 ||
         encoding == JSON ||
         encoding == NMEA0183;
}

[[nodiscard]]
inline bool isSupportedEncoding(const std::string & encoding) noexcept
{
  return isSupportedEncoding(std::string_view{encoding});
}

[[nodiscard]]
inline bool isGnssEncoding(const std::string & encoding) noexcept
{
  return isGnssEncoding(std::string_view{encoding});
}

[[nodiscard]]
inline bool isTextEncoding(const std::string & encoding) noexcept
{
  return isTextEncoding(std::string_view{encoding});
}

}  // namespace usb_application_encodings
}  // namespace usb_msgs

#endif  // USB_MSGS__USB_APPLICATION_ENCODINGS_HPP_
