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

#include "standard_robot_pp_ros2/sbus_protocol.hpp"

#include <algorithm>
#include <cmath>

namespace standard_robot_pp_ros2
{

SbusEncoder::SbusEncoder() {}

uint16_t SbusEncoder::mapValueToChannel(float value, float max_value)
{
  //[-max_value, max_value]  [0, 2047]
  //1024

  value = std::clamp(value, -max_value, max_value);
  
  // [-1, 1]
  float normalized = value / max_value;
  
  //  [0, 2047]//1024
  uint16_t channel_value = static_cast<uint16_t>(
    SBUS_CENTER_VALUE + normalized * (SBUS_MAX_VALUE - SBUS_CENTER_VALUE)
  );
  
  // 
  return std::clamp(channel_value, SBUS_MIN_VALUE, SBUS_MAX_VALUE);
}

void SbusEncoder::packChannels(const uint16_t channels[SBUS_NUM_CHANNELS], uint8_t * output)
{
  // SBUS Э�飺16 ��ͨ����ÿ��ͨ�� 11 λ������� 22 �ֽ�
  // ͨ�����ݰ�λ�����С����
  
  output[0] = static_cast<uint8_t>(channels[0] & 0x07FF);
  output[1] = static_cast<uint8_t>((channels[0] & 0x07FF) >> 8 | (channels[1] & 0x07FF) << 3);
  output[2] = static_cast<uint8_t>((channels[1] & 0x07FF) >> 5 | (channels[2] & 0x07FF) << 6);
  output[3] = static_cast<uint8_t>((channels[2] & 0x07FF) >> 2);
  output[4] = static_cast<uint8_t>((channels[2] & 0x07FF) >> 10 | (channels[3] & 0x07FF) << 1);
  output[5] = static_cast<uint8_t>((channels[3] & 0x07FF) >> 7 | (channels[4] & 0x07FF) << 4);
  output[6] = static_cast<uint8_t>((channels[4] & 0x07FF) >> 4 | (channels[5] & 0x07FF) << 7);
  output[7] = static_cast<uint8_t>((channels[5] & 0x07FF) >> 1);
  output[8] = static_cast<uint8_t>((channels[5] & 0x07FF) >> 9 | (channels[6] & 0x07FF) << 2);
  output[9] = static_cast<uint8_t>((channels[6] & 0x07FF) >> 6 | (channels[7] & 0x07FF) << 5);
  output[10] = static_cast<uint8_t>((channels[7] & 0x07FF) >> 3);
  output[11] = static_cast<uint8_t>(channels[8] & 0x07FF);
  output[12] = static_cast<uint8_t>((channels[8] & 0x07FF) >> 8 | (channels[9] & 0x07FF) << 3);
  output[13] = static_cast<uint8_t>((channels[9] & 0x07FF) >> 5 | (channels[10] & 0x07FF) << 6);
  output[14] = static_cast<uint8_t>((channels[10] & 0x07FF) >> 2);
  output[15] = static_cast<uint8_t>((channels[10] & 0x07FF) >> 10 | (channels[11] & 0x07FF) << 1);
  output[16] = static_cast<uint8_t>((channels[11] & 0x07FF) >> 7 | (channels[12] & 0x07FF) << 4);
  output[17] = static_cast<uint8_t>((channels[12] & 0x07FF) >> 4 | (channels[13] & 0x07FF) << 7);
  output[18] = static_cast<uint8_t>((channels[13] & 0x07FF) >> 1);
  output[19] = static_cast<uint8_t>((channels[13] & 0x07FF) >> 9 | (channels[14] & 0x07FF) << 2);
  output[20] = static_cast<uint8_t>((channels[14] & 0x07FF) >> 6 | (channels[15] & 0x07FF) << 5);
  output[21] = static_cast<uint8_t>((channels[15] & 0x07FF) >> 3);
}

std::vector<uint8_t> SbusEncoder::encode(const SbusFrame & frame)
{
  std::vector<uint8_t> data(SBUS_FRAME_SIZE);
  
  // ��ʼ�ֽ�
  data[0] = SBUS_START_BYTE;
  
  // ��� 16 ��ͨ�����ݣ�22 �ֽڣ�
  packChannels(frame.channels, &data[1]);
  
  // ��־λ
  data[23] = frame.flags;
  
  // �����ֽ�
  data[24] = SBUS_END_BYTE;
  
  return data;
}

std::vector<uint8_t> SbusEncoder::invertBytes(const std::vector<uint8_t> & data)
{
  std::vector<uint8_t> inverted(data.size());
  for (size_t i = 0; i < data.size(); ++i) {
    inverted[i] = ~data[i];
  }
  return inverted;
}

}  // namespace standard_robot_pp_ros2