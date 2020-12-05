// -*-c++-*--------------------------------------------------------------------
// Copyright 2020 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include "cam_sync.h"

#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::placeholders;

namespace cam_sync_ros2
{
CamSync::CamSync(const rclcpp::NodeOptions & options)
: Node("cam_sync", options)
{
  const auto qos = rmw_qos_profile_default;
  for (size_t cam_idx = 0; cam_idx < 2; cam_idx++) {
    const std::string cam = "cam_" + std::to_string(cam_idx);
    imageSubs_.push_back(std::make_shared<message_filters::Subscriber<Image>>(
      this, cam + "/image_raw", qos));
    cameraInfoSubs_.push_back(
      std::make_shared<message_filters::Subscriber<CameraInfo>>(
        this, cam + "/camera_info", qos));
    imagePubs_.push_back(image_transport::create_publisher(
      this, cam + "/synced/image_raw", rmw_qos_profile_default));
    cameraInfoPubs_.push_back(
      create_publisher<CameraInfo>(cam + "/synced/camera_info", 3));
  }
  sync2_ = std::make_shared<Sync2>(
    Sync2Policy(10), *imageSubs_[0], *cameraInfoSubs_[0], *imageSubs_[1],
    *cameraInfoSubs_[1]);
  sync2_->registerCallback(
    std::bind(&CamSync::callback2, this, _1, _2, _3, _4));
}

CamSync::~CamSync() {}

void CamSync::callback2(
  const Image::ConstSharedPtr & im0c, const CameraInfo::ConstSharedPtr & ci0c,
  const Image::ConstSharedPtr & im1c, const CameraInfo::ConstSharedPtr & ci1c)
{
  // brutal hack: cast away the const-ness and modify the time stamp of
  // the original message. Just hope that the message_filters will not
  // look at the timestamps after the callback...
  // If you know of a better way to do this without making a copy of the image,
  // let me know.
  const rclcpp::Time t0 =
    message_filters::message_traits::TimeStamp<CameraInfo>::value(*ci0c);
  const rclcpp::Time t1 =
    message_filters::message_traits::TimeStamp<CameraInfo>::value(*ci1c);
  if (t1 < t0) {
    auto im0 = std::const_pointer_cast<Image>(im0c);
    auto ci0 = std::const_pointer_cast<CameraInfo>(ci0c);
    im0->header.stamp = t1;
    ci0->header.stamp = t1;
  } else if (t1 > t0) {
    auto im1 = std::const_pointer_cast<Image>(im1c);
    auto ci1 = std::const_pointer_cast<CameraInfo>(ci1c);
    im1->header.stamp = t0;
    ci1->header.stamp = t0;
  }

  cameraInfoPubs_[0]->publish(*ci0c);
  cameraInfoPubs_[1]->publish(*ci1c);
  imagePubs_[0].publish(im0c);
  imagePubs_[1].publish(im1c);
}

}  // namespace cam_sync_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(cam_sync_ros2::CamSync)
