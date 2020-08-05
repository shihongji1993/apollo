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
 * @brief Defines the EsdCanClient class which inherits CanClient.
 */

#pragma once

#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_client/usb_can/controlcan.h"
#include "modules/drivers/canbus/common/canbus_consts.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"

/**
 * @namespace apollo::drivers::canbus::can
 * @brief apollo::drivers::canbus::can
 */
namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

class UsbCanClient : public CanClient {
 public:
  // Initialize the USB CAN client by specified CAN card parameters.
  // parameter CAN card parameters to initialize the CAN client.
  // return True If the initialization is successful.
  bool Init(const CANCardParameter &parameter) override;

  virtual ~UsbCanClient();

  // Start the USB CAN client.
  // return the status of the start action which is defined by
  // apollo::common::ErrorCode.
  apollo::common::ErrorCode Start() override;

  void Stop() override;

  // function: Send messages
  // param：frames The messages to send.
  // param：frame_num The amount of messages to send.
  // return the status of the sending action which is defined by
  // apollo::common::ErrorCode.
  apollo::common::ErrorCode Send(const std::vector<CanFrame> &frames,
                                 int32_t *const frame_num) override;

  /**
   * @brief Receive messages
   * @param frames The messages to receive.
   * @param frame_num The amount of messages to receive.
   * @return The status of the receiving action which is defined by
   *         apollo::common::ErrorCode.
   */
  apollo::common::ErrorCode Receive(std::vector<CanFrame> *const frames,
                                    int32_t *const frame_num) override;

  /**
   * @brief Get the error string.
   * @param status The status to get the error string.
   */
  std::string GetErrorString(const int32_t status) override;

 private:
  CANCardParameter::CANChannelId port_;              // the port
  VCI_CAN_OBJ send_frames_[MAX_CAN_SEND_FRAME_LEN];  // the data to send
  VCI_CAN_OBJ
  recv_frames_[MAX_CAN_RECV_FRAME_LEN];    // save the data received by Can
  VCI_BOARD_INFO pInfo_;                   // save the broad info。
  VCI_BOARD_INFO pInfo1_[MAX_SERIAL_MUM];  // save the broad info。
  int num_ = 0;                            // the num of device。
  DWORD dev_handler_ = 0;
  const DWORD dev_index_ = 0;
  const DWORD can_index_ = 0;
  const DWORD reserved_para_ = 0;
};

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
