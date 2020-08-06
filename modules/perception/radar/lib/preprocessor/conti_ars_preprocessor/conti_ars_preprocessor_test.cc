/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/radar/lib/preprocessor/conti_ars_preprocessor/conti_ars_preprocessor.h"

#include "gtest/gtest.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace radar {

using common::Header;
using drivers::CubRadarObs;

class ContiArsPreprocessorTest : public testing::Test {
 public:
  void SetMaxRadarIdx() { preprocessor.current_idx_ = MAX_RADAR_IDX - 1; }

 protected:
  ContiArsPreprocessor preprocessor;
};

TEST_F(ContiArsPreprocessorTest, init) {
  float delay_time = static_cast<float>(preprocessor.GetDelayTime());
  EXPECT_FLOAT_EQ(delay_time, 0.0);
  FLAGS_work_root =
      "/apollo/modules/perception/testdata/"
      "radar/preprocessor";
  bool init_result = preprocessor.Init();
  EXPECT_TRUE(init_result);
  delay_time = static_cast<float>(preprocessor.GetDelayTime());
  EXPECT_FLOAT_EQ(delay_time, 0.07);
  EXPECT_EQ(preprocessor.Name(), "ContiArsPreprocessor");
}

TEST_F(ContiArsPreprocessorTest, preprocess) {
  drivers::CubRadar raw_obs;
  Header radar_header;
  radar_header.set_timestamp_sec(151237772.355345434);
  radar_header.set_radar_timestamp(151237772355345434);
  radar_header.set_module_name("radar");
  radar_header.set_sequence_num(0);
  raw_obs.mutable_header()->CopyFrom(radar_header);

  PreprocessorOptions options;
  drivers::CubRadar corrected_obs;

  CubRadarObs* cub_obs = raw_obs.add_contiobs();
  cub_obs->set_obstacle_id(80);
  cub_obs->set_meas_state(2);
  Header obj_header;
  obj_header.set_timestamp_sec(151237772.305345434);
  obj_header.set_radar_timestamp(151237772305345434);
  obj_header.set_module_name("radar");
  obj_header.set_sequence_num(0);
  cub_obs->mutable_header()->CopyFrom(obj_header);

  cub_obs = raw_obs.add_contiobs();
  cub_obs->set_obstacle_id(0);
  cub_obs->set_meas_state(2);
  obj_header.set_timestamp_sec(151237772.375345434);
  obj_header.set_radar_timestamp(151237772375345434);
  cub_obs->mutable_header()->CopyFrom(obj_header);

  cub_obs = raw_obs.add_contiobs();
  cub_obs->set_obstacle_id(1);
  cub_obs->set_meas_state(0);
  obj_header.set_timestamp_sec(151237772.385345434);
  obj_header.set_radar_timestamp(151237772385345434);
  cub_obs->mutable_header()->CopyFrom(obj_header);

  cub_obs = raw_obs.add_contiobs();
  cub_obs->set_obstacle_id(2);
  cub_obs->set_meas_state(3);
  obj_header.set_timestamp_sec(151237772.385345434);
  obj_header.set_radar_timestamp(151237772385345434);
  cub_obs->mutable_header()->CopyFrom(obj_header);

  cub_obs = raw_obs.add_contiobs();
  cub_obs->set_obstacle_id(0);
  cub_obs->set_meas_state(2);
  obj_header.set_timestamp_sec(151237772.585345434);
  obj_header.set_radar_timestamp(151237772385345434);
  cub_obs->mutable_header()->CopyFrom(obj_header);

  preprocessor.Preprocess(raw_obs, options, &corrected_obs);

  EXPECT_EQ(corrected_obs.contiobs_size(), 3);
  EXPECT_EQ(corrected_obs.contiobs(0).obstacle_id(), 1);
  EXPECT_EQ(corrected_obs.contiobs(1).obstacle_id(), 2);
  EXPECT_EQ(corrected_obs.contiobs(2).obstacle_id(), 3);

  raw_obs.clear_contiobs();
  corrected_obs.clear_contiobs();

  radar_header.set_timestamp_sec(151237772.425345434);
  radar_header.set_radar_timestamp(151237772425345434);
  radar_header.set_module_name("radar");
  radar_header.set_sequence_num(0);
  raw_obs.mutable_header()->CopyFrom(radar_header);

  cub_obs = raw_obs.add_contiobs();
  cub_obs->set_obstacle_id(1);
  cub_obs->set_meas_state(1);
  obj_header.set_timestamp_sec(151237772.445345434);
  obj_header.set_radar_timestamp(151237772445345434);
  cub_obs->mutable_header()->CopyFrom(obj_header);

  cub_obs = raw_obs.add_contiobs();
  cub_obs->set_obstacle_id(2);
  cub_obs->set_meas_state(2);
  obj_header.set_timestamp_sec(151237772.455345434);
  obj_header.set_radar_timestamp(151237772455345434);
  cub_obs->mutable_header()->CopyFrom(obj_header);

  cub_obs = raw_obs.add_contiobs();
  cub_obs->set_obstacle_id(3);
  cub_obs->set_meas_state(1);
  obj_header.set_timestamp_sec(151237772.455345434);
  obj_header.set_radar_timestamp(151237772455345434);
  cub_obs->mutable_header()->CopyFrom(obj_header);

  preprocessor.Preprocess(raw_obs, options, &corrected_obs);

  EXPECT_EQ(corrected_obs.contiobs_size(), 3);
  EXPECT_EQ(corrected_obs.contiobs(0).obstacle_id(), 4);
  EXPECT_EQ(corrected_obs.contiobs(1).obstacle_id(), 3);
  EXPECT_EQ(corrected_obs.contiobs(2).obstacle_id(), 5);

  raw_obs.clear_contiobs();
  corrected_obs.clear_contiobs();

  radar_header.set_timestamp_sec(151237772.485345434);
  radar_header.set_radar_timestamp(151237772485345434);
  radar_header.set_module_name("radar");
  radar_header.set_sequence_num(0);
  raw_obs.mutable_header()->CopyFrom(radar_header);

  SetMaxRadarIdx();
  cub_obs = raw_obs.add_contiobs();
  cub_obs->set_obstacle_id(50);
  cub_obs->set_meas_state(1);
  obj_header.set_timestamp_sec(151237772.525345434);
  obj_header.set_radar_timestamp(151237772525345434);
  cub_obs->mutable_header()->CopyFrom(obj_header);

  preprocessor.Preprocess(raw_obs, options, &corrected_obs);
  EXPECT_EQ(corrected_obs.contiobs_size(), 1);
  EXPECT_EQ(corrected_obs.contiobs(0).obstacle_id(), 1);
}

}  // namespace radar
}  // namespace perception
}  // namespace apollo
