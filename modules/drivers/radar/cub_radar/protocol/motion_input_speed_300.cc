/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/drivers/radar/cub_radar/protocol/motion_input_speed_300.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace drivers {
namespace cub_radar {

using apollo::drivers::CubRadar;
using apollo::drivers::canbus::Byte;

const uint32_t MotionInputSpeed300::ID = 0x08;

MotionInputSpeed300::MotionInputSpeed300() {}
MotionInputSpeed300::~MotionInputSpeed300() {}

uint32_t MotionInputSpeed300::GetPeriod() const {
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

/**
 * @brief update the data
 * @param data a pointer to the data to be updated
 */
void MotionInputSpeed300::UpdateData(uint8_t* data) {
  SetLifecount(data);
  SetSpeedData(data);
  SetDirection(data);
  SetMagicID(data);
  count_++;
  AINFO << std::hex << (int)(data[0]) << " " << (int)(data[1]) << " "
        << (int)(data[2]) << " " << (int)(data[3]) << " " << (int)(data[4])
        << " " << (int)(data[5]) << " " << (int)(data[6]) << " "
        << (int)(data[7]);
}

void MotionInputSpeed300::SetLifecount(uint8_t* data) {
  Byte frame(data);
  frame.set_value(static_cast<unsigned char>((count_ & 0xFF00) >> 8), 0, 8);
  Byte frame_2(data + 1);
  frame_2.set_value(static_cast<unsigned char>(count_ & 0x00FF), 0, 8);
}

void MotionInputSpeed300::SetSpeedData(uint8_t* data) {
  int speed_change = (int)(speed_ / 0.1);
  Byte frame_low(data + 2);
  frame_low.set_value(static_cast<unsigned char>((speed_change & 0xFF00) >> 8),
                      0, 8);

  Byte frame_high(data + 3);
  frame_high.set_value(static_cast<unsigned char>(speed_change & 0x00FF), 0, 8);
}

void MotionInputSpeed300::SetDirection(uint8_t* data) {
  if (std::isnan(speed_)) {
    AWARN << "speed is nan";
    return;
  }
  int speed_direction = 1;
  if (fabs(speed_) < 0.02) {
    speed_direction = 1;
  } else if (speed_ < 0) {
    speed_direction = 0;
  } else {
    speed_direction = 1;
  }
  Byte frame(data + 4);
  frame.set_value(speed_direction, 0, 8);

  Byte frame_high(data + 5);
  frame_high.set_value(0, 0, 8);
}

void MotionInputSpeed300::SetMagicID(uint8_t* data) {
  Byte frame_high(data + 6);
  frame_high.set_value(0xCA, 0, 8);

  Byte frame_low(data + 7);
  frame_low.set_value(0xFF, 0, 8);
}

/**
 * @brief reset the private variables
 */
void MotionInputSpeed300::Reset() { speed_ = NAN; }

void MotionInputSpeed300::SetSpeed(const float& speed) { speed_ = speed; }

}  // namespace cub_radar
}  // namespace drivers
}  // namespace apollo
