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

#include <string>
#ifndef LOGGING_H_
#define LOGGING_H_

namespace cam_sync_ros2
{
#define BOMB_OUT(...)                    \
  {                                      \
    ROS_ERROR_STREAM(__VA_ARGS__);       \
    std::stringstream SS;                \
    SS << __VA_ARGS__;                   \
    throw(std::runtime_error(SS.str())); \
  }
#define LOG_INFO(...)                              \
  {                                                \
    RCLCPP_INFO_STREAM(get_logger(), __VA_ARGS__); \
  }
#define LOG_WARN(...)                              \
  {                                                \
    RCLCPP_WARN_STREAM(get_logger(), __VA_ARGS__); \
  }
#define LOG_ERROR(...)                              \
  {                                                 \
    RCLCPP_ERROR_STREAM(get_logger(), __VA_ARGS__); \
  }

}  // namespace cam_sync_ros2
#endif  // LOGGING_H_
