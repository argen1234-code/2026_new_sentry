// Copyright 2025 SMBU-PolarBear-Robotics-Team
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

#ifndef STANDARD_ROBOT_PP_ROS2__SBUS_PROTOCOL_HPP_
#define STANDARD_ROBOT_PP_ROS2__SBUS_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace standard_robot_pp_ros2
{

// SBUS §¿?ø@??
constexpr uint8_t SBUS_START_BYTE = 0x0F;
constexpr uint8_t SBUS_END_BYTE = 0x00;
constexpr size_t SBUS_FRAME_SIZE = 25;
constexpr size_t SBUS_NUM_CHANNELS = 16;
constexpr uint16_t SBUS_MIN_VALUE = 0;
constexpr uint16_t SBUS_MAX_VALUE = 2047;
constexpr uint16_t SBUS_CENTER_VALUE = 1024;

// SBUS ???¦Ë
constexpr uint8_t SBUS_FLAG_CH17 = 0x01;
constexpr uint8_t SBUS_FLAG_CH18 = 0x02;
constexpr uint8_t SBUS_FLAG_FRAME_LOST = 0x04;
constexpr uint8_t SBUS_FLAG_FAILSAFE = 0x08;

/**
 * @brief SBUS ???????
 */
struct SbusFrame
{
  uint8_t start_byte;                    // 0x0F
  uint16_t channels[SBUS_NUM_CHANNELS];  // 16 ?????????? 11 ¦Ë
  uint8_t flags;                         // ???¦Ë
  uint8_t end_byte;                      // 0x00
};

/**
 * @brief SBUS ????????
 */
class SbusEncoder
{
public:
  SbusEncoder();
  
  /**
   * @brief ????????? SBUS ????
   * @param value ????
   * @param max_value ???????
   * @return SBUS ???? (0-2047)
   */
  static uint16_t mapValueToChannel(float value, float max_value);
  
  /**
   * @brief ???? SBUS ?
   * @param frame SBUS ???
   * @return ????????????? (25 ???)
   */
  static std::vector<uint8_t> encode(const SbusFrame & frame);
  
  /**
   * @brief ????????SBUS ??????
   * @param data ??????
   * @return ??????????
   */
  static std::vector<uint8_t> invertBytes(const std::vector<uint8_t> & data);
  
private:
  /**
   * @brief ?? 16 ?? 11 ¦Ë????????? 22 ???
   * @param channels ???????
   * @param output ??????????
   */
  static void packChannels(const uint16_t channels[SBUS_NUM_CHANNELS], uint8_t * output);
};

}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__SBUS_PROTOCOL_HPP_