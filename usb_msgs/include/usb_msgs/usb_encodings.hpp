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

#ifndef USB_MSGS__USB_ENCODINGS_HPP_
#define USB_MSGS__USB_ENCODINGS_HPP_

#include <string>

namespace usb_msgs
{
namespace usb_encodings
{
const char RAW[] = "raw";
const char GZIPPED[] = "gzipped";
const char ZLIB[] = "zlib";

// Utility functions for encoding
static inline bool isRawEncoding(const std::string & encoding)
{
  return encoding == RAW;
}

static inline bool isGzippedEncoding(const std::string & encoding)
{
  return encoding == GZIPPED;
}

static inline bool isZlibEncoding(const std::string & encoding)
{
  return encoding == ZLIB;
}

}  // namespace usb_encodings
}  // namespace usb_msgs

#endif  // USB_MSGS__USB_ENCODINGS_HPP_
