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

#include "pointcloud_checker/pointcloud_checker_node.hpp"

#include <memory>
#include <utility>

namespace pointcloud_checker
{
PointcloudCheckerNode::PointcloudCheckerNode(const rclcpp::NodeOptions & node_options)
: Node("pointcloud_checker", node_options)
{
  using std::placeholders::_1;
  // get parameter
  update_hz_ = this->declare_parameter<int>("update_hz", 10);
  error_point_num_ = this->declare_parameter<int>("error_point_num", 50);

  // publisher
  pub_pcd_result = this->create_publisher<BoolStamped>("~/output/pointcloud_check", 1);

  // subscriber
  sub_pcd_ = this->create_subscription<PointCloud2>(
    "~/input/pointcloud", 1, std::bind(&PointcloudCheckerNode::onPointcloud, this, _1));

  /* Diagnostic Updater */
  updater_ptr_ = std::make_shared<Updater>(this, 1.0 / update_hz_);
  updater_ptr_->setHardwareID("pointcloud_checker");
  updater_ptr_->add("pointcloud_checker", this, &PointcloudCheckerNode::monitorPointcloudNum);
}

// function for diagnostics
void PointcloudCheckerNode::monitorPointcloudNum(DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;

  // TODO consider the gear

  if (!pcd_ready_) {
    stat.summary(DiagStatus::OK, "Pointcloud data does not subscribed yet");
    return;
  }

  if (pcd_point_num_ < error_point_num_) {
    stat.summary(DiagStatus::OK, "OK");
    return;
  }
  stat.summary(DiagStatus::WARN, " Pointcloud check failed");
}

void PointcloudCheckerNode::onPointcloud(const PointCloud2::ConstSharedPtr msg)
{
  pcd_point_num_ = msg->data.size();
  pcd_ready_ = true;

  // publish pcd result
  BoolStamped pcd_num_result;
  pcd_num_result.stamp = msg->header.stamp;
  pcd_num_result.data = (pcd_point_num_ < error_point_num_);
  pub_pcd_result->publish(pcd_num_result);
}

}  // namespace pointcloud_checker

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_checker::PointcloudCheckerNode)
