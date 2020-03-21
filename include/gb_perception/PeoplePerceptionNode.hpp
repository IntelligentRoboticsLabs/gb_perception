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

#ifndef GB_PERCEPTION__PEOPLEPERCEPTIONNODE_HPP_
#define GB_PERCEPTION__PEOPLEPERCEPTIONNODE_HPP_

#include <mutex>
#include <string>
#include <memory>
#include <vector>

#include "cv_bridge/cv_bridge.h"

#include "openpose_detector/OpenPoseDetector.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace gb_perception
{

class PeoplePerceptionNode : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  explicit PeoplePerceptionNode(const std::string & name);

private:
  bool active_;
  std::mutex mutex_;
  rclcpp::TimerBase::SharedPtr timer_;

  openpose_detector::OpenPoseDetector detector_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr openpose_result_pub_;
  cv_bridge::CvImagePtr last_image_;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & previous_state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void perception_loop();

  void print_openpose_result(
    const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> op_data);
  void publish_openpose_result(
    const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>> op_data);
};

}  // namespace gb_perception

#endif  // GB_PERCEPTION__PEOPLEPERCEPTIONNODE_HPP_
