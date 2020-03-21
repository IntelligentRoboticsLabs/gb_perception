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

#include <stdlib.h>

#include <memory>
#include <string>
#include <vector>

#include "openpose_detector/OpenPoseDetector.hpp"

#include "openpose/flags.hpp"
#include "openpose/headers.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace openpose_detector
{

OpenPoseDetector::OpenPoseDetector()
: hands_active_(false),
  face_active_(false)
{
  init();
}

void
OpenPoseDetector::init()
{
  opWrapper_ = std::make_shared<op::Wrapper>(op::ThreadManagerMode::Asynchronous);

  // We can ask for openpose, as it is a ament package
  std::string pkgpath = ament_index_cpp::get_package_share_directory("gb_perception");
  std::string modelspath = pkgpath + "/../../../openpose/share/OpenPose/models";

  const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
  const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
  const auto faceNetInputSize = op::flagsToPoint(
    op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
  const auto handNetInputSize = op::flagsToPoint(op::String(
        FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
  const auto poseMode = op::flagsToPoseMode(FLAGS_body);
  const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
  const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
  // heatmaps to add
  const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
    FLAGS_heatmaps_add_PAFs);
  const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
  // >1 camera view?
  const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
  // Face and hand detectors
  const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
  const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
  // Enabling Google Logging
  const bool enableGoogleLogging = true;

  // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
  const op::WrapperStructPose wrapperStructPose{
    poseMode, netInputSize, outputSize, keypointScaleMode, FLAGS_num_gpu, FLAGS_num_gpu_start,
    FLAGS_scale_number, static_cast<float>(FLAGS_scale_gap),
    op::flagsToRenderMode(FLAGS_render_pose, multipleView), poseModel, !FLAGS_disable_blending,
    static_cast<float>(FLAGS_alpha_pose), static_cast<float>(FLAGS_alpha_heatmap),
    FLAGS_part_to_show, op::String(modelspath), heatMapTypes, heatMapScaleMode,
    FLAGS_part_candidates, static_cast<float>(FLAGS_render_threshold), FLAGS_number_people_max,
    FLAGS_maximize_positives, FLAGS_fps_max, op::String(FLAGS_prototxt_path),
    op::String(FLAGS_caffemodel_path), static_cast<float>(FLAGS_upsampling_ratio),
    enableGoogleLogging};
  opWrapper_->configure(wrapperStructPose);
  // Face configuration (use op::WrapperStructFace{} to disable it)
  const op::WrapperStructFace wrapperStructFace{
    face_active_, faceDetector, faceNetInputSize,
    op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
    static_cast<float>(FLAGS_face_alpha_pose), static_cast<float>(FLAGS_face_alpha_heatmap),
    static_cast<float>(FLAGS_face_render_threshold)};
  opWrapper_->configure(wrapperStructFace);
  // Hand configuration (use op::WrapperStructHand{} to disable it)
  const op::WrapperStructHand wrapperStructHand{
    hands_active_, handDetector, handNetInputSize, FLAGS_hand_scale_number,
    static_cast<float>(FLAGS_hand_scale_range), op::flagsToRenderMode(FLAGS_hand_render,
      multipleView, FLAGS_render_pose), static_cast<float>(FLAGS_hand_alpha_pose),
    static_cast<float>(FLAGS_hand_alpha_heatmap),
    static_cast<float>(FLAGS_hand_render_threshold)};
  opWrapper_->configure(wrapperStructHand);
  // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
  const op::WrapperStructExtra wrapperStructExtra{
    FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads};
  opWrapper_->configure(wrapperStructExtra);
  // Output (comment or use default argument to disable any output)
  const op::WrapperStructOutput wrapperStructOutput{
    FLAGS_cli_verbose, op::String(FLAGS_write_keypoint),
    op::stringToDataFormat(FLAGS_write_keypoint_format),
    op::String(FLAGS_write_json), op::String(FLAGS_write_coco_json),
    FLAGS_write_coco_json_variants, FLAGS_write_coco_json_variant,
    op::String(FLAGS_write_images), op::String(FLAGS_write_images_format),
    op::String(FLAGS_write_video), FLAGS_write_video_fps, FLAGS_write_video_with_audio,
    op::String(FLAGS_write_heatmaps), op::String(FLAGS_write_heatmaps_format),
    op::String(FLAGS_write_video_3d),
    op::String(FLAGS_write_video_adam), op::String(FLAGS_write_bvh), op::String(FLAGS_udp_host),
    op::String(FLAGS_udp_port)};
  opWrapper_->configure(wrapperStructOutput);
  // No GUI. Equivalent to: opWrapper_->configure(op::WrapperStructGui{});
  // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
  if (FLAGS_disable_multi_thread) {
    opWrapper_->disableMultiThreading();
  }

  opWrapper_->disableMultiThreading();
  opWrapper_->start();
}

const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>
OpenPoseDetector::process_image(const cv::Mat & cvImageToProcess)
{
  const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(cvImageToProcess);
  return opWrapper_->emplaceAndPop(imageToProcess);
}

void
OpenPoseDetector::set_hands_activation(bool active)
{
  if (active != hands_active_) {
    hands_active_ = active;
    opWrapper_->stop();
    init();
  }
}

void
OpenPoseDetector::set_face_activation(bool active)
{
  if (active != face_active_) {
    face_active_ = active;
    opWrapper_->stop();
    init();
  }
}

}  // namespace openpose_detector
