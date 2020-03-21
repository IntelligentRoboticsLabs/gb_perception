// Copyright 2019 Intelligent Robotics Lab
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

#include <string>
#include <vector>
#include <regex>
#include <iostream>
#include <memory>
#include <map>

#include "cv_bridge/cv_bridge.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "openpose_detector/OpenPoseDetector.hpp"
#include "gb_perception/PeoplePerceptionNode.hpp"
#include "openpose/headers.hpp"

#include "rclcpp/rclcpp.hpp"

#include "gtest/gtest.h"


TEST(openpose_detector, detect)
{
  std::string pkgpath = ament_index_cpp::get_package_share_directory("gb_perception");
  std::string imgpath = pkgpath + "/test_images/image_1.jpg";

  openpose_detector::OpenPoseDetector detector;

  const cv::Mat cvImageToProcess = cv::imread(imgpath);
  auto datumsPtr = detector.process_image(cvImageToProcess);

  ASSERT_NE(datumsPtr, nullptr);
  ASSERT_FALSE(detector.are_hands_active());
  ASSERT_FALSE(detector.is_face_active());

  ASSERT_EQ(datumsPtr->size(), 1u);

  const auto & poseKeypoints = datumsPtr->at(0)->poseKeypoints;
  ASSERT_EQ(poseKeypoints.getSize(0), 1);
  ASSERT_EQ(poseKeypoints.getSize(1), 25);
  ASSERT_EQ(poseKeypoints.getSize(2), 3);

  for (auto person = 0; person < poseKeypoints.getSize(0); person++) {
    for (auto bodyPart = 0; bodyPart < poseKeypoints.getSize(1); bodyPart++) {
      for (auto xyscore = 0; xyscore < poseKeypoints.getSize(2); xyscore++) {
        auto value = poseKeypoints[{person, bodyPart, xyscore}];
        ASSERT_NE(value, 0);
      }
    }
  }

  const auto & faceKeypoints = datumsPtr->at(0)->faceKeypoints;
  ASSERT_EQ(faceKeypoints.getSize(0), 0);
  ASSERT_EQ(faceKeypoints.getSize(1), 0);
  ASSERT_EQ(faceKeypoints.getSize(2), 0);

  ASSERT_EQ(datumsPtr->at(0)->handKeypoints.size(), 2u);

  const auto & leftHandKeypoints = datumsPtr->at(0)->handKeypoints[0];
  const auto & rightHandKeypoints = datumsPtr->at(0)->handKeypoints[1];

  ASSERT_EQ(leftHandKeypoints.getSize(0), 0);
  ASSERT_EQ(leftHandKeypoints.getSize(1), 0);
  ASSERT_EQ(leftHandKeypoints.getSize(2), 0);
  ASSERT_EQ(rightHandKeypoints.getSize(0), 0);
  ASSERT_EQ(rightHandKeypoints.getSize(1), 0);
  ASSERT_EQ(rightHandKeypoints.getSize(2), 0);

  detector.set_face_activation(true);
  ASSERT_FALSE(detector.are_hands_active());
  ASSERT_TRUE(detector.is_face_active());
  datumsPtr = detector.process_image(cvImageToProcess);

  const auto & faceKeypoints_2 = datumsPtr->at(0)->faceKeypoints;
  ASSERT_EQ(faceKeypoints_2.getSize(0), 1);
  ASSERT_EQ(faceKeypoints_2.getSize(1), 70);
  ASSERT_EQ(faceKeypoints_2.getSize(2), 3);

  ASSERT_EQ(datumsPtr->at(0)->handKeypoints.size(), 2u);

  const auto & leftHandKeypoints_2 = datumsPtr->at(0)->handKeypoints[0];
  const auto & rightHandKeypoints_2 = datumsPtr->at(0)->handKeypoints[1];

  ASSERT_EQ(leftHandKeypoints_2.getSize(0), 0);
  ASSERT_EQ(leftHandKeypoints_2.getSize(1), 0);
  ASSERT_EQ(leftHandKeypoints_2.getSize(2), 0);
  ASSERT_EQ(rightHandKeypoints_2.getSize(0), 0);
  ASSERT_EQ(rightHandKeypoints_2.getSize(1), 0);
  ASSERT_EQ(rightHandKeypoints_2.getSize(2), 0);

  detector.set_hands_activation(true);
  ASSERT_TRUE(detector.are_hands_active());
  ASSERT_TRUE(detector.is_face_active());
  datumsPtr = detector.process_image(cvImageToProcess);

  const auto & faceKeypoints_3 = datumsPtr->at(0)->faceKeypoints;
  ASSERT_EQ(faceKeypoints_3.getSize(0), 1);
  ASSERT_EQ(faceKeypoints_3.getSize(1), 70);
  ASSERT_EQ(faceKeypoints_3.getSize(2), 3);

  ASSERT_EQ(datumsPtr->at(0)->handKeypoints.size(), 2u);

  const auto & leftHandKeypoints_3 = datumsPtr->at(0)->handKeypoints[0];
  const auto & rightHandKeypoints_3 = datumsPtr->at(0)->handKeypoints[1];

  ASSERT_EQ(leftHandKeypoints_3.getSize(0), 1);
  ASSERT_EQ(leftHandKeypoints_3.getSize(1), 21);
  ASSERT_EQ(leftHandKeypoints_3.getSize(2), 3);
  ASSERT_EQ(rightHandKeypoints_3.getSize(0), 1);
  ASSERT_EQ(rightHandKeypoints_3.getSize(1), 21);
  ASSERT_EQ(rightHandKeypoints_3.getSize(2), 3);
}


TEST(openpose_detector_node, people_detection)
{
  auto perception_node = std::make_shared<gb_perception::PeoplePerceptionNode>("perception_node");
  auto image_node = rclcpp::Node::make_shared("image_node");
  auto image_pub = image_node->create_publisher<sensor_msgs::msg::Image>("image_in", 10);

  std::string pkgpath = ament_index_cpp::get_package_share_directory("gb_perception");
  std::string imgpath = pkgpath + "/test_images/image_1.jpg";

  cv_bridge::CvImage cv_image;
  cv_image.image = cv::imread(imgpath);
  cv_image.header.frame_id = "camera";
  cv_image.header.stamp = image_node->now();
  cv_image.encoding = "bgr8";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(perception_node->get_node_base_interface());
  executor.add_node(image_node);

  perception_node->add_activation("people_perception_node");
  perception_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  {
    rclcpp::Rate rate(10);
    auto start = perception_node->now();
    while ((perception_node->now() - start).seconds() < 0.2) {
      executor.spin_some();
      rate.sleep();
    }
  }

  perception_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  {
    rclcpp::Rate rate(10);
    auto start = perception_node->now();
    while ((perception_node->now() - start).seconds() < 0.5) {
      image_pub->publish(*cv_image.toImageMsg());
      executor.spin_some();
      rate.sleep();
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
