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

#ifndef OPENPOSE_DETECTOR__OPENPOSEDETECTOR_HPP_
#define OPENPOSE_DETECTOR__OPENPOSEDETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <openpose/headers.hpp>

#include <memory>
#include <vector>

namespace openpose_detector
{

class OpenPoseDetector
{
public:
  OpenPoseDetector();

  void init();

  const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>
  process_image(const cv::Mat & cvImageToProcess);

  void set_hands_activation(bool active);
  void set_face_activation(bool active);

  bool are_hands_active() {return hands_active_;}
  bool is_face_active() {return face_active_;}

private:
  std::shared_ptr<op::Wrapper> opWrapper_;

  bool hands_active_;
  bool face_active_;
};

}  // namespace openpose_detector

#endif  // OPENPOSE_DETECTOR__OPENPOSEDETECTOR_HPP_
