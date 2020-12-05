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

#ifndef CAM_SYNC_
#define CAM_SYNC_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <image_transport/image_transport.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

namespace cam_sync_ros2
{
class CamSync : public rclcpp::Node
{
public:
  typedef sensor_msgs::msg::Image Image;
  typedef std::shared_ptr<const Image> ImageConstPtr;
  typedef sensor_msgs::msg::CameraInfo CameraInfo;
  explicit CamSync(const rclcpp::NodeOptions & options);
  ~CamSync();

private:
  void callback2(
    const Image::ConstSharedPtr & im0, const CameraInfo::ConstSharedPtr & ci0,
    const Image::ConstSharedPtr & im1, const CameraInfo::ConstSharedPtr & ci1);
  typedef message_filters::sync_policies::ApproximateTime<
    Image, CameraInfo, Image, CameraInfo>
    Sync2Policy;
  typedef message_filters::Synchronizer<Sync2Policy> Sync2;
  // ------------ variables ------------
  std::vector<std::shared_ptr<message_filters::Subscriber<Image>>> imageSubs_;
  std::vector<std::shared_ptr<message_filters::Subscriber<CameraInfo>>>
    cameraInfoSubs_;
  std::shared_ptr<Sync2> sync2_;
  std::vector<image_transport::Publisher> imagePubs_;
  std::vector<rclcpp::Publisher<CameraInfo>::SharedPtr> cameraInfoPubs_;
};
}  // namespace cam_sync_ros2
#endif  // CAM_SYNC_H_
