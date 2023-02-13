// Copyright 2022 Tier IV, Inc.
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

#ifndef POINTCLOUD_CHECKER__POINTCLOUD_CHECKER_NODE_HPP_
#define POINTCLOUD_CHECKER__POINTCLOUD_CHECKER_NODE_HPP_

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tier4_debug_msgs/msg/bool_stamped.hpp"

#include <memory>

namespace pointcloud_checker
{
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;
using sensor_msgs::msg::PointCloud2;
using tier4_debug_msgs::msg::BoolStamped;

class PointcloudCheckerNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<BoolStamped>::SharedPtr pub_pcd_result;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_pcd_;

  PointCloud2::ConstSharedPtr pointcloud_ptr_;
  double update_hz_;
  int error_point_num_;
  int pcd_point_num_ = 0;
  bool pcd_ready_ = false;

  // Diagnostic Updater
  std::shared_ptr<Updater> updater_ptr_;

  void onPointcloud(const PointCloud2::ConstSharedPtr msg);
  void monitorPointcloudNum(DiagnosticStatusWrapper & stat);

public:
  explicit PointcloudCheckerNode(const rclcpp::NodeOptions & node_options);
};
}  // namespace pointcloud_checker

#endif  // POINTCLOUD_CHECKER__POINTCLOUD_CHECKER_NODE_HPP_
