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
 * @file esd_can_client.cc
 * @brief the encapsulate call the api of esd can card according to can_client.h
 *interface
 **/
#include "modules/drivers/canbus/can_client/usb_can/usb_can_client.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/sensor_gflags.h"

namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

VCI_BOARD_INFO pInfo;       // save the broad info。
VCI_BOARD_INFO pInfo1[50];  //
int num = 0;                // the num of device。

using apollo::common::ErrorCode;

bool UsbCanClient::Init(const CANCardParameter &parameter) {
  if (!parameter.has_channel_id()) {
    AERROR << "Init CAN failed: parameter does not have channel id. The "
              "parameter is "
           << parameter.DebugString();
    return false;
  } else {
    port_ = parameter.channel_id();
    return true;
  }
}

UsbCanClient::~UsbCanClient() {
  if (dev_handler_) {
    Stop();
  }
}

ErrorCode UsbCanClient::Start() {
  if (is_started_) {
    return ErrorCode::OK;
  }

  if (port_ > MAX_CAN_PORT || port_ < 0) {
    AERROR << "can port number [" << port_ << "] is out of the range [0,"
           << MAX_CAN_PORT << "]";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  num = VCI_FindUsbDevice2(pInfo1);
  dev_handler_ = VCI_OpenDevice(VCI_USBCAN2, 0, 0);
  if (dev_handler_ == 0)  // open device
  {
    AERROR << "Open device error code: " << dev_handler_;
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  AINFO << "Open device succ code: " << dev_handler_;

  dev_handler_ = VCI_ReadBoardInfo(VCI_USBCAN2, 0, &pInfo);
  if (dev_handler_ != 0)  // read the Board Info。
  {
    printf(">>Get VCI_ReadBoardInfo success!\n");
  }

  VCI_INIT_CONFIG config;
  config.AccCode = 0;
  config.AccMask = 0xFFFFFFFF;  // set the mask
  config.Filter = 1;
  config.Timing0 = 0x01; /*250 Kbps  0x03  0x1C*/
  config.Timing1 = 0x1C;
  config.Mode = 0;  // normal mode

  dev_handler_ = VCI_InitCAN(VCI_USBCAN2, 0, 0, &config);
  if (dev_handler_ != 1) {
    printf(">>Init CAN1 error\n");
    VCI_CloseDevice(VCI_USBCAN2, 0);
  }
  dev_handler_ = VCI_StartCAN(VCI_USBCAN2, 0, 0);
  if (dev_handler_ != 1) {
    printf(">>Start CAN1 error\n");
    VCI_CloseDevice(VCI_USBCAN2, 0);
  }

  dev_handler_ = VCI_InitCAN(VCI_USBCAN2, 0, 1, &config);
  if (dev_handler_ != 1) {
    printf(">>Init CAN2 error\n");
    VCI_CloseDevice(VCI_USBCAN2, 0);
  }

  dev_handler_ = VCI_StartCAN(VCI_USBCAN2, 0, 1);
  if (dev_handler_ != 1) {
    printf(">>Start CAN2 error\n");
    VCI_CloseDevice(VCI_USBCAN2, 0);
  }

  is_started_ = true;
  return ErrorCode::OK;
}

void UsbCanClient::Stop() {
  if (is_started_) {
    is_started_ = false;
    dev_handler_ = VCI_CloseDevice(VCI_USBCAN2, 0);  // close device。
    if (dev_handler_ != 1) {
      AERROR << "close usbcan fail!the port:" << port_;
    } else {
      AERROR << "close usbcan success";
    }
  }
}

// Synchronous transmission of CAN messages
ErrorCode UsbCanClient::Send(const std::vector<CanFrame> &frames,
                             int32_t *const frame_num) {
  if (frame_num == nullptr) {
    AERROR << "frame_num pointer is null";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  if (static_cast<size_t>(*frame_num) != frames.size()) {
    AERROR << "frame num is incorrect.";
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }
  if (!is_started_) {
    AERROR << "Esd can client has not been initiated! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }

  for (size_t i = 0; i < frames.size(); i++) {
    send_frames_[0].ID = frames[i].id;
    send_frames_[0].SendType = 0;
    send_frames_[0].RemoteFlag = 0;
    send_frames_[0].ExternFlag = 1;
    send_frames_[0].DataLen = frames[i].len;
    std::memcpy(send_frames_[0].Data, frames[i].data, frames[i].len);
    int senlen = VCI_Transmit(VCI_USBCAN2, 0, 0, send_frames_, 1);
    if (senlen != 1) {
      AERROR << "send message failed" << std::endl;
      return ErrorCode::CAN_CLIENT_ERROR_BASE;
    }
    // Synchronous transmission of CAN messages
  }
  return ErrorCode::OK;
}

ErrorCode UsbCanClient::Receive(std::vector<CanFrame> *const frames,
                                int32_t *const frame_num) {
  if (!is_started_) {
    AERROR << "Esd can client is not init! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
  }

  if (*frame_num > MAX_CAN_RECV_FRAME_LEN || *frame_num < 0) {
    AERROR << "recv can frame num not in range[0, " << MAX_CAN_RECV_FRAME_LEN
           << "], frame_num:" << *frame_num;
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }
  int ret = VCI_Receive(VCI_USBCAN2, 0, 0, recv_frames_, 3000, 100);

  if (ret == 0) {
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  for (int i = 0; i < ret; i++) {
    CanFrame cf;
    cf.id = recv_frames_[i].ID;
    cf.len = recv_frames_[i].DataLen;
    std::memcpy(cf.data, recv_frames_[i].Data, recv_frames_[i].DataLen);
    cf.timestamp.tv_sec = recv_frames_[i].TimeStamp / 10000;
    cf.timestamp.tv_usec = recv_frames_[i].TimeStamp % 10000;
    frames->push_back(cf);
    *frame_num = frames->size();
  }
  return ErrorCode::OK;
}

std::string UsbCanClient::GetErrorString(const int32_t /*status*/) {
  return "";
}

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
