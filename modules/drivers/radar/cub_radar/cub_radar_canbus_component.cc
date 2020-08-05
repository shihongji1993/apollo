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
 * @file
 */

#include "modules/drivers/radar/cub_radar/cub_radar_canbus_component.h"

#include "Eigen/Geometry"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/proto/sensor_canbus_conf.pb.h"
#include "modules/drivers/proto/cub_radar.pb.h"
#include "modules/drivers/radar/cub_radar/cub_radar_message_manager.h"

/**
 * @namespace apollo::drivers::cub_radar
 * @brief apollo::drivers
 */
namespace apollo {
namespace drivers {
namespace cub_radar {

using apollo::common::ErrorCode;
using apollo::drivers::canbus::CanClientFactory;
using apollo::drivers::canbus::SenderMessage;
using apollo::drivers::canbus::SensorCanbusConf;
using apollo::localization::LocalizationEstimate;

CubRadarCanbusComponent::CubRadarCanbusComponent()
    : monitor_logger_buffer_(
          apollo::common::monitor::MonitorMessageItem::CUB_RADAR) {}
CubRadarCanbusComponent::~CubRadarCanbusComponent() { Stop(); }

bool CubRadarCanbusComponent::Init() {
  if (!GetProtoConfig(&cub_radar_conf_)) {
    return OnError("Unable to load canbus conf file: " + ConfigFilePath());
  }

  AINFO << "The canbus conf file is loaded: " << ConfigFilePath();
  ADEBUG << "Canbus_conf:" << cub_radar_conf_.ShortDebugString();

  // Init can client
  auto can_factory = CanClientFactory::Instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(
      cub_radar_conf_.can_conf().can_card_parameter());
  if (!can_client_) {
    return OnError("Failed to create can client.");
  }
  AINFO << "Can client is successfully created.";
  cub_radar_writer_ =
      node_->CreateWriter<CubRadar>(cub_radar_conf_.radar_channel());
  pose_reader_ = node_->CreateReader<LocalizationEstimate>(
      FLAGS_localization_topic,
      [&](const std::shared_ptr<LocalizationEstimate>& pose) {
        PoseCallback(pose);
      });

  /*=====test send message==============
    int speed =50;
    AINFO << "radar speed:" << speed << ";yaw rate:";
    MotionInputSpeed300 input_speed;
    input_speed.SetSpeed(speed);
    SenderMessage<CubRadar> sender_message_speed(MotionInputSpeed300::ID,
                                                 &input_speed);
    sender_message_speed.Update();
    can_client_->SendSingleFrame({sender_message_speed.CanFrame()});
  //=====test send message==============*/

  sensor_message_manager_ = std::unique_ptr<CubRadarMessageManager>(
      new CubRadarMessageManager(cub_radar_writer_));
  if (sensor_message_manager_ == nullptr) {
    return OnError("Failed to create message manager.");
  }
  sensor_message_manager_->set_radar_conf(cub_radar_conf_.radar_conf());
  sensor_message_manager_->set_can_client(can_client_);
  AINFO << "Sensor message manager is successfully created.";

  if (can_receiver_.Init(can_client_.get(), sensor_message_manager_.get(),
                         enable_log_) != ErrorCode::OK) {
    return OnError("Failed to init can receiver.");
  }
  AINFO << "The can receiver is successfully initialized.";

  start_success_ = Start();
  return start_success_;
}

// apollo::common::ErrorCode CubRadarCanbusComponent::ConfigureRadar() {
//   RadarConfig200 radar_config;
//   radar_config.set_radar_conf(cub_radar_conf_.radar_conf());
//   SenderMessage<CubRadar> sender_message(RadarConfig200::ID, &radar_config);
//   sender_message.Update();
//   return can_client_->SendSingleFrame({sender_message.CanFrame()});
// }

// apollo::common::ErrorCode CubRadarCanbusComponent::SetVecilotyRadar() {
//   RadarConfig200 radar_config;
//   radar_config.set_radar_conf(cub_radar_conf_.radar_conf());
//   SenderMessage<CubRadar> sender_message(RadarConfig200::ID, &radar_config);
//   sender_message.Update();
//   return can_client_->SendSingleFrame({sender_message.CanFrame()});
// }

bool CubRadarCanbusComponent::Start() {
  // 1. init and start the can card hardware
  if (can_client_->Start() != ErrorCode::OK) {
    return OnError("Failed to start can client");
  }
  AINFO << "Can client is started.";

  //=====don't need configradar===
  // if (ConfigureRadar() != ErrorCode::OK) {
  //   return OnError("Failed to configure radar.");
  // }
  // AINFO << "The radar is successfully configured.";

  // 2. start receive first then send
  if (can_receiver_.Start() != ErrorCode::OK) {
    return OnError("Failed to start can receiver.");
  }
  AINFO << "Can receiver is started.";

  // last step: publish monitor messages
  monitor_logger_buffer_.INFO("Canbus is started.");
  return true;
}

void CubRadarCanbusComponent::Stop() {
  if (start_success_) {
    can_receiver_.Stop();
    can_client_->Stop();
  }
}

// Send the error to monitor and return it
bool CubRadarCanbusComponent::OnError(const std::string& error_msg) {
  monitor_logger_buffer_.ERROR(error_msg);
  AERROR << error_msg;
  return false;
}

void CubRadarCanbusComponent::PoseCallback(
    const std::shared_ptr<LocalizationEstimate>& pose_msg) {
  uint64_t send_interval = 20000000;
  uint64_t now_nsec = cyber::Time().Now().ToNanosecond();
  if (last_nsec_ != 0 && (now_nsec - last_nsec_) < send_interval) {
    return;
  }
  last_nsec_ = now_nsec;
  Eigen::Quaterniond orientation_vehicle_world(
      pose_msg->pose().orientation().qw(), pose_msg->pose().orientation().qx(),
      pose_msg->pose().orientation().qy(), pose_msg->pose().orientation().qz());
  Eigen::Matrix3d rotation_matrix =
      orientation_vehicle_world.toRotationMatrix().inverse();
  Eigen::Vector3d speed_v(pose_msg->pose().linear_velocity().x(),
                          pose_msg->pose().linear_velocity().y(),
                          pose_msg->pose().linear_velocity().z());
  float speed = static_cast<float>((rotation_matrix * speed_v).y());
  float yaw_rate = static_cast<float>(pose_msg->pose().angular_velocity().z() *
                                      180.0f / M_PI);

  AINFO << "radar speed:" << speed << ";yaw rate:" << yaw_rate;
  MotionInputSpeed300 input_speed;
  input_speed.SetSpeed(speed);
  SenderMessage<CubRadar> sender_message_speed(MotionInputSpeed300::ID,
                                               &input_speed);
  sender_message_speed.Update();
  can_client_->SendSingleFrame({sender_message_speed.CanFrame()});

  // don't have yaw spped to input
  // MotionInputYawRate301 input_yawrate;
  // input_yawrate.SetYawRate(yaw_rate);
  // SenderMessage<CubRadar> sender_message_yawrate(MotionInputYawRate301::ID,
  //                                                &input_yawrate);
  // sender_message_yawrate.Update();
  // can_client_->SendSingleFrame({sender_message_yawrate.CanFrame()});
}

}  // namespace cub_radar
}  // namespace drivers
}  // namespace apollo
