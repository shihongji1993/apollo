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

#pragma once
// #include <SFML/Graphics.hpp>

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/drivers/proto/cub_radar.pb.h"

namespace apollo {
namespace drivers {
namespace cub_radar {

using apollo::drivers::CubRadar;

class ClusterGeneralInfo100
    : public apollo::drivers::canbus::ProtocolData<CubRadar> {
 public:
  static const uint32_t ID;
  ClusterGeneralInfo100();
  void Parse(const std::uint8_t* bytes, int32_t length,
             CubRadar* cub_radar) const override;

 private:
  int ObstacleId(const std::uint8_t* bytes, int32_t length) const;

  // int counting_time(const std::uint8_t* bytes, int32_t length) const;

  // double velocity(const std::uint8_t* bytes, int32_t length) const;

  double LongitudePosition(const std::uint8_t* bytes, int32_t length) const;

  double LateralPosition(const std::uint8_t* bytes, int32_t length) const;

  double LateryVelocity(const std::uint8_t* bytes, int32_t length) const;

  double LongitudeVelocity(const std::uint8_t* bytes, int32_t length) const;

  // double lateral_vel(const std::uint8_t* bytes, int32_t length) const;

  // double rcs(const std::uint8_t* bytes, int32_t length) const;

  // int dynprop(const std::uint8_t* bytes, int32_t length) const;
  // int visual() {
  //   sf::RenderWindow window(sf::VideoMode(200, 200), "SFML works!");
  //   sf::CircleShape shape(100.f);
  //   shape.setFillColor(sf::Color::Green);

  //   while (window.isOpen()) {
  //     sf::Event event;
  //     while (window.pollEvent(event)) {
  //       if (event.type == sf::Event::Closed) window.close();
  //     }

  //     window.clear();
  //     window.draw(shape);
  //     window.display();
  //   }

  //   return 0;
  // }
};

}  // namespace cub_radar
}  // namespace drivers
}  // namespace apollo
