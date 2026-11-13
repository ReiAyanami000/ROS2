// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <memory>
#include <string>
#include "nav2_core/global_planner.hpp"
#include "transport_orthogonal_planner/search.hpp"

namespace transport_orthogonal_planner
{
class OrthogonalPlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override {active_ = true;}
  void deactivate() override {active_ = false;}
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;
private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  Options options_;
  bool active_{false};
};
}  // namespace transport_orthogonal_planner
