/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/radar/cub_radar/protocol/cluster_general_info_100.h"

#include <iostream>
#include <string>

#include "glog/logging.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"
#include "modules/drivers/radar/cub_radar/protocol/const_vars.h"
namespace apollo {
namespace drivers {
namespace cub_radar {

using apollo::drivers::canbus::Byte;

ClusterGeneralInfo100::ClusterGeneralInfo100() {}
const uint32_t ClusterGeneralInfo100::ID = 0x100;

void ClusterGeneralInfo100::Parse(const std::uint8_t* bytes, int32_t length,
                                  CubRadar* cub_radar) const {
  auto obs = cub_radar->add_contiobs();
  obs->set_clusterortrack(true);
  obs->set_obstacle_id(ObstacleId(bytes, length));
  // obs->set_counting_time(counting_time(bytes, length));
  obs->set_longitude_vel(LongitudeVelocity(bytes, length));
  obs->set_lateral_vel(LateryVelocity(bytes, length));
  obs->set_longitude_dist(LongitudePosition(bytes, length));
  obs->set_lateral_dist(LateralPosition(bytes, length));
  // obs->set_longitude_dist_rms((double)(0.0));
  // obs->set_lateral_dist_rms((double)(0.0));
  // obs->set_longitude_vel_rms((double)(0.0));
  // obs->set_lateral_vel_rms((double)(0.0));
  // obs->set_probexist((double)(1.0));
  // obs->set_dynprop((int)(0));    // state of cluster
  // obs->set_rcs((double)(15.0));  // Radar Cross section

  double timestamp = apollo::common::time::Clock::NowInSeconds();
  auto header = obs->mutable_header();
  header->CopyFrom(cub_radar->header());
  header->set_timestamp_sec(timestamp);
  // AINFO << (cub_radar->header().timestamp_sec());
  // printf << ObstacleId(bytes, length) << std::endl;
  AINFO << std::hex << (int)(bytes[0]) << " " << (int)(bytes[1]) << " "
        << (int)(bytes[2]) << " " << (int)(bytes[3]) << " " << (int)(bytes[4])
        << " " << (int)(bytes[5]) << " " << (int)(bytes[6]) << " "
        << (int)(bytes[7])
        << " x_veciloty: " << std::to_string(LateryVelocity(bytes, length))
        << " y_veciloty: " << std::to_string(LongitudeVelocity(bytes, length))
        << " x_position:" << std::to_string(LateralPosition(bytes, length))
        << " y_position:" << std::to_string(LongitudePosition(bytes, length))
        << " id " << ObstacleId(bytes, length);
}

int ClusterGeneralInfo100::ObstacleId(const std::uint8_t* bytes,
                                      int32_t length) const {
  Byte t0(bytes + 1);
  uint32_t x = t0.get_byte(0, 8);
  int ret = x;
  return ret;
}

double ClusterGeneralInfo100::LateryVelocity(const std::uint8_t* bytes,
                                             int32_t length) const {
  Byte t0(bytes + 2);
  uint32_t x = t0.get_byte(0, 8);
  Byte t1(bytes + 3);
  uint32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;
  double ret;
  ret = InverseCode(x);
  return ret * 0.1;
}

double ClusterGeneralInfo100::LongitudeVelocity(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 3);
  uint32_t x = t0.get_byte(0, 4);

  Byte t1(bytes + 4);
  uint32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;
  double ret;
  ret = InverseCode(x);
  return ret * 0.1;
}

double ClusterGeneralInfo100::LateralPosition(const std::uint8_t* bytes,
                                              int32_t length) const {
  Byte t0(bytes + 5);
  uint32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 6);
  uint32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;
  double ret = InverseCode(x);
  return ret * 0.1;
}

double ClusterGeneralInfo100::LongitudePosition(const std::uint8_t* bytes,
                                                int32_t length) const {
  Byte t0(bytes + 6);
  uint32_t x = t0.get_byte(0, 4);
  Byte t1(bytes + 7);
  uint32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;
  double ret;
  ret = InverseCode(x);
  return ret * 0.1;
}

double ClusterGeneralInfo100::InverseCode(uint32_t x) const {
  double ret;
  if (x > 0b100000000000) {
    ret = 0XFFF - x + 1;
    ret = -ret;
  } else {
    ret = x;
  }
  return ret;
}
}  // namespace cub_radar
}  // namespace drivers
}  // namespace apollo
