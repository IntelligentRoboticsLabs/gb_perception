// Copyright 2020 Intelligent Robotics Lab
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
#include <memory>
#include <vector>
#include <map>

#include "cv_bridge/cv_bridge.h"

#include "openpose_detector/OpenPoseDetector.hpp"
#include "gb_perception/PeoplePerceptionNode.hpp"

namespace gb_perception
{

using std::placeholders::_1;

PeoplePerceptionNode::PeoplePerceptionNode(const std::string & name)
: CascadeLifecycleNode(name),
  last_image_(nullptr)
{
  image_sub_ = create_subscription<sensor_msgs::msg::Image>("image_in", 1,
      std::bind(&PeoplePerceptionNode::image_callback, this, _1));
  openpose_result_pub_ = create_publisher<sensor_msgs::msg::Image>("openpose_image_out", 10);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PeoplePerceptionNode::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  timer_ = nullptr;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PeoplePerceptionNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  active_ = true;

  timer_ = create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&PeoplePerceptionNode::perception_loop, this));
  openpose_result_pub_->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PeoplePerceptionNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  active_ = false;
  timer_ = nullptr;
  openpose_result_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void
PeoplePerceptionNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  if (!active_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  last_image_ = cv_bridge::toCvCopy(msg, "rgb8");
}

void
PeoplePerceptionNode::perception_loop()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (last_image_ != nullptr) {
    auto result = detector_.process_image(last_image_->image);
    publish_openpose_result(result);
    print_openpose_result(result);
  }
}

void
PeoplePerceptionNode::publish_openpose_result(
  const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> op_data)
{
  if (openpose_result_pub_->get_subscription_count() > 0) {
    cv_bridge::CvImage cv_image;
    cv_image.image = OP_OP2CVCONSTMAT(op_data->at(0)->cvOutputData);
    cv_image.header.frame_id = "camera";
    cv_image.header.stamp = now();
    cv_image.encoding = "bgr8";
    openpose_result_pub_->publish(*cv_image.toImageMsg());
  }
}

void
PeoplePerceptionNode::print_openpose_result(
  const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> op_data)
{
  std::map<unsigned int, std::string> POSE_BODY_25_BODY_PARTS {
    {0, "Nose"},
    {1, "Neck"},
    {2, "RShoulder"},
    {3, "RElbow"},
    {4, "RWrist"},
    {5, "LShoulder"},
    {6, "LElbow"},
    {7, "LWrist"},
    {8, "MidHip"},
    {9, "RHip"},
    {10, "RKnee"},
    {11, "RAnkle"},
    {12, "LHip"},
    {13, "LKnee"},
    {14, "LAnkle"},
    {15, "REye"},
    {16, "LEye"},
    {17, "REar"},
    {18, "LEar"},
    {19, "LBigToe"},
    {20, "LSmallToe"},
    {21, "LHeel"},
    {22, "RBigToe"},
    {23, "RSmallToe"},
    {24, "RHeel"},
    {25, "Background"}
  };

  if (op_data != nullptr && !op_data->empty()) {
    const auto & poseKeypoints = op_data->at(0)->poseKeypoints;
    std::cout << "=======================================================" << std::endl;
    std::cout << "Person pose keypoints:" << std::endl;
    for (auto person = 0; person < poseKeypoints.getSize(0); person++) {
      std::cout << "Person " + std::to_string(person) + " (x, y, score):" << std::endl;
      for (auto bodyPart = 0; bodyPart < poseKeypoints.getSize(1); bodyPart++) {
        std::string valueToPrint;
        for (auto xyscore = 0; xyscore < poseKeypoints.getSize(2); xyscore++) {
          valueToPrint += std::to_string(
            poseKeypoints[{person, bodyPart, xyscore}]) + " ";
        }
        std::cout << POSE_BODY_25_BODY_PARTS[bodyPart] << " = " << valueToPrint << std::endl;
      }
    }
    std::cout << " " << std::endl;
  } else {
    std::cout << "Nullptr or empty datumsPtr found." << std::endl;
  }
}

}  // namespace gb_perception
