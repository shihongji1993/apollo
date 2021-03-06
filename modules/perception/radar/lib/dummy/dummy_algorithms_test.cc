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
#include "modules/perception/radar/lib/dummy/dummy_algorithms.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace radar {

using drivers::CubRadar;

class DummyAlgorithmsTest : public testing::Test {
 protected:
  DummyPreprocessor preprocessor;
  DummyDetector detector;
  DummyRoiFilter roi_filter;
};

CubRadar MockContiObs() {
  CubRadar raw_obs;

  drivers::CubRadarObs cub_obs;
  cub_obs.set_clusterortrack(0);
  cub_obs.set_obstacle_id(80);
  cub_obs.set_longitude_dist(20);
  cub_obs.set_lateral_dist(10);
  cub_obs.set_longitude_vel(10);
  cub_obs.set_lateral_vel(5);
  cub_obs.set_rcs(15);
  cub_obs.set_dynprop(0);
  cub_obs.set_probexist(0.8);
  cub_obs.set_longitude_dist_rms(0.2);
  cub_obs.set_lateral_dist_rms(0.1);
  cub_obs.set_longitude_vel_rms(0.2);
  cub_obs.set_lateral_vel_rms(0.1);
  cub_obs.set_oritation_angle(10);
  cub_obs.set_oritation_angle_rms(2.0);
  cub_obs.set_length(2.0);
  cub_obs.set_width(1.0);
  cub_obs.set_obstacle_class(CONTI_CAR);
  cub_obs.set_meas_state(2);

  raw_obs.add_contiobs()->CopyFrom(cub_obs);
  cub_obs.set_obstacle_class(CONTI_TRUCK);
  raw_obs.add_contiobs()->CopyFrom(cub_obs);
  cub_obs.set_obstacle_class(CONTI_PEDESTRIAN);
  raw_obs.add_contiobs()->CopyFrom(cub_obs);
  cub_obs.set_obstacle_class(CONTI_MOTOCYCLE);
  raw_obs.add_contiobs()->CopyFrom(cub_obs);
  cub_obs.set_obstacle_class(CONTI_BICYCLE);
  raw_obs.add_contiobs()->CopyFrom(cub_obs);
  cub_obs.set_obstacle_class(CONTI_TYPE_UNKNOWN);
  raw_obs.add_contiobs()->CopyFrom(cub_obs);

  return raw_obs;
}

TEST_F(DummyAlgorithmsTest, dummy_test) {
  CubRadar raw_obs = MockContiObs();
  CubRadar corrected_obs;
  PreprocessorOptions preprocessor_options;
  bool init_result = preprocessor.Init();
  EXPECT_TRUE(init_result);
  EXPECT_EQ(preprocessor.Name(), "DummyPreprocessor");
  preprocessor.Preprocess(raw_obs, preprocessor_options, &corrected_obs);
  EXPECT_EQ(corrected_obs.contiobs_size(), 6);
  EXPECT_EQ(corrected_obs.contiobs(0).obstacle_id(), 80);
  EXPECT_EQ(corrected_obs.contiobs(0).meas_state(), 2);

  DetectorOptions detector_options;
  init_result = detector.Init();
  EXPECT_TRUE(init_result);
  EXPECT_EQ(detector.Name(), "DummyDetector");

  base::FramePtr detected_frame(new base::Frame);
  detector.Detect(corrected_obs, detector_options, detected_frame);
  Eigen::Vector3d center(20, 10, 0);
  Eigen::Vector3f velocity(10, 5, 0);
  EXPECT_LT((center - detected_frame->objects[0]->center).norm(), 1.0e-6);
  EXPECT_LT((velocity - detected_frame->objects[0]->velocity).norm(), 1.0e-6);

  RoiFilterOptions roi_filter_options;
  init_result = roi_filter.Init();
  EXPECT_TRUE(init_result);
  EXPECT_EQ(roi_filter.Name(), "DummyRoiFilter");
  bool roi_filter_result =
      roi_filter.RoiFilter(roi_filter_options, detected_frame);
  EXPECT_TRUE(roi_filter_result);
  EXPECT_EQ(detected_frame->objects.size(), 6);
}

}  // namespace radar
}  // namespace perception
}  // namespace apollo
