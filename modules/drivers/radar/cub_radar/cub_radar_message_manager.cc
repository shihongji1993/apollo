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

/**
 * @file cub_radar_message_manager.h
 * @brief The class of CubRadarMessageManager
 */

#include "modules/drivers/radar/cub_radar/cub_radar_message_manager.h"

#include "modules/common/util/message_util.h"
#include "modules/drivers/radar/cub_radar/protocol/cluster_general_info_100.h"
// #include "modules/drivers/radar/cub_radar/protocol/cluster_list_status_600.h"
// #include "modules/drivers/radar/cub_radar/protocol/object_list_status_60a.h"
// #include "modules/drivers/radar/cub_radar/protocol/radar_state_201.h"

// #include
// "modules/drivers/radar/cub_radar/protocol/cluster_quality_info_702.h"
// #include
// "modules/drivers/radar/cub_radar/protocol/object_extended_info_60d.h"
// #include
// "modules/drivers/radar/cub_radar/protocol/object_general_info_60b.h"
// #include
// "modules/drivers/radar/cub_radar/protocol/object_quality_info_60c.h"

namespace apollo {
namespace drivers {
namespace cub_radar {

using Clock = apollo::common::time::Clock;
using micros = std::chrono::microseconds;
using apollo::cyber::Writer;
using apollo::drivers::canbus::CanClient;
using apollo::drivers::canbus::ProtocolData;
using apollo::drivers::canbus::SenderMessage;

CubRadarMessageManager::CubRadarMessageManager(
    const std::shared_ptr<Writer<CubRadar>> &writer)
    : cub_radar_writer_(writer) {
  // AddRecvProtocolData<RadarState201, true>();
  // AddRecvProtocolData<ClusterListStatus600, true>();
  AddRecvProtocolData<ClusterGeneralInfo100, true>();
  // AddRecvProtocolData<ClusterQualityInfo702, true>();
  // AddRecvProtocolData<ObjectExtendedInfo60D, true>();
  // AddRecvProtocolData<ObjectGeneralInfo60B, true>();
  // AddRecvProtocolData<ObjectListStatus60A, true>();
  // AddRecvProtocolData<ObjectQualityInfo60C, true>();
}

void CubRadarMessageManager::set_radar_conf(RadarConf radar_conf) {
  radar_config_.set_radar_conf(radar_conf);
}

void CubRadarMessageManager::set_can_client(
    std::shared_ptr<CanClient> can_client) {
  can_client_ = can_client;
}

ProtocolData<CubRadar> *CubRadarMessageManager::GetMutableProtocolDataById(
    const uint32_t message_id) {
  uint32_t converted_message_id = message_id;
  if (protocol_data_map_.find(converted_message_id) ==
      protocol_data_map_.end()) {
    ADEBUG << "Unable to get protocol data because of invalid message_id:"
           << message_id;
    return nullptr;
  }
  return protocol_data_map_[converted_message_id];
}

void CubRadarMessageManager::Parse(const uint32_t message_id,
                                   const uint8_t *data, int32_t length) {
  uint32_t id_object = 0x100;
  if (message_id <= 0x100) {
    id_object = message_id;
  }
  ProtocolData<CubRadar> *sensor_protocol_data =
      GetMutableProtocolDataById(id_object);
  if (sensor_protocol_data == nullptr) {
    return;
  }
  std::lock_guard<std::mutex> lock(sensor_data_mutex_);

  sensor_protocol_data->Parse(data, length, &sensor_data_);

  received_ids_.insert(id_object);
  // check if need to check period
  const auto it = check_ids_.find(id_object);
  if (it != check_ids_.end()) {
    const int64_t time = absl::ToUnixMicros(Clock::Now());
    it->second.real_period = time - it->second.last_time;
    // if period 1.5 large than base period, inc error_count
    const double period_multiplier = 1.5;
    if (it->second.real_period >
        (static_cast<double>(it->second.period) * period_multiplier)) {
      it->second.error_count += 1;
    } else {
      it->second.error_count = 0;
    }
    it->second.last_time = time;
  }
}

}  // namespace cub_radar
}  // namespace drivers
}  // namespace apollo
