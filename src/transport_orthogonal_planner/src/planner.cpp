// SPDX-License-Identifier: Apache-2.0
#include "transport_orthogonal_planner/planner.hpp"
#include <mutex>
#include <utility>
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace transport_orthogonal_planner
{
void OrthogonalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  costmap_ros_ = std::move(costmap_ros);
  const auto node = node_.lock();
  if (!node || !costmap_ros_) {throw std::runtime_error("Missing planner resources");}
  auto read_double = [&](const std::string & key, double value) {
      nav2_util::declare_parameter_if_not_declared(node, name+"."+key, rclcpp::ParameterValue(value));
      const double result=node->get_parameter(name+"."+key).as_double();
      if (!std::isfinite(result) || result<0) {throw std::runtime_error("Invalid "+key);}
      return result;
    };
  auto read_limit = [&](const std::string & key, int value) {
      nav2_util::declare_parameter_if_not_declared(node, name+"."+key, rclcpp::ParameterValue(value));
      const auto result=node->get_parameter(name+"."+key).as_int();
      if (result<=0 || result>10000000) {throw std::runtime_error("Invalid "+key);}
      return static_cast<size_t>(result);
    };
  options_.turn_penalty = read_double("turn_penalty", 0.10);
  options_.cost_weight = read_double("cost_weight", 2.0);
  options_.timeout = read_double("max_planning_time", 3.0);
  options_.max_expansions = read_limit("max_expansions", 1000000);
  options_.max_states = read_limit("max_states", 2000000);
  if (options_.timeout<=0) {throw std::runtime_error("max_planning_time must be positive");}
  RCLCPP_INFO(node->get_logger(), "Configured orthogonal planner '%s'", name.c_str());
}

void OrthogonalPlanner::cleanup()
{
  active_ = false;
  costmap_ros_.reset();
  node_.reset();
}

nav_msgs::msg::Path OrthogonalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  const auto node=node_.lock();
  if (!active_ || !node || !costmap_ros_) {
    throw nav2_core::PlannerException("Orthogonal planner is not active");
  }
  const auto frame=costmap_ros_->getGlobalFrameID();
  if (start.header.frame_id!=frame || goal.header.frame_id!=frame) {
    throw nav2_core::PlannerTFError("Start and goal must be in the costmap global frame");
  }
  if (!costmap_ros_->getUseRadius()) {
    throw nav2_core::PlannerException("This version requires robot_radius, not a polygon footprint");
  }
  auto valid_pose = [](const geometry_msgs::msg::Pose & pose) {
      const auto & p=pose.position;
      const auto & q=pose.orientation;
      return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
             std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
             std::isfinite(q.w) && std::abs(p.z)<1e-6 &&
             std::abs(q.x)<1e-6 && std::abs(q.y)<1e-6 &&
             std::hypot(q.z,q.w)>1e-9;
    };
  if (!valid_pose(start.pose) || !valid_pose(goal.pose)) {
    throw nav2_core::PlannerException("Expected finite planar poses with valid quaternions");
  }
  Options options=options_;
  options.radius=costmap_ros_->getRobotRadius();
  for (const auto & p : costmap_ros_->getRobotFootprint()) {
    options.radius=std::max(options.radius,std::hypot(p.x,p.y));
  }
  Grid grid;
  // Snapshot under the costmap mutex; search never blocks costmap updates.
  {
    auto * map=costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    const auto width=map->getSizeInCellsX(), height=map->getSizeInCellsY();
    if (width==0 || height==0 || width>100000 || height>100000 ||
      static_cast<size_t>(width)*height > options.max_states/5)
    {throw nav2_core::PlannerException("Map is empty or exceeds configured state budget");}
    grid.width=static_cast<int>(width);
    grid.height=static_cast<int>(height);
    grid.resolution=map->getResolution();
    grid.origin_x=map->getOriginX(); grid.origin_y=map->getOriginY();
    grid.costs.assign(map->getCharMap(), map->getCharMap()+static_cast<size_t>(width)*height);
  }
  try {
    Search search(grid, options, cancel_checker);
    const Result result=search.plan(
      {start.pose.position.x,start.pose.position.y},
      {goal.pose.position.x,goal.pose.position.y});
    nav_msgs::msg::Path path;
    path.header.frame_id=frame;
    path.header.stamp=node->now();
    for (size_t i=0; i<result.points.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header=path.header;
      pose.pose.position.x=result.points[i].x;
      pose.pose.position.y=result.points[i].y;
      if (i+1<result.points.size()) {
        const double yaw=std::atan2(result.points[i+1].y-result.points[i].y,
                                   result.points[i+1].x-result.points[i].x);
        pose.pose.orientation.z=std::sin(yaw/2);
        pose.pose.orientation.w=std::cos(yaw/2);
      } else {
        const double norm=std::hypot(goal.pose.orientation.z,goal.pose.orientation.w);
        pose.pose.orientation.z=goal.pose.orientation.z/norm;
        pose.pose.orientation.w=goal.pose.orientation.w/norm;
      }
      path.poses.push_back(pose);
    }
    RCLCPP_INFO(node->get_logger(), "Orthogonal plan: %zu points, %zu expansions",
                path.poses.size(), result.expansions);
    return path;
  } catch (const SearchError & error) {
    switch (error.code) {
      case ErrorCode::StartOutside: throw nav2_core::StartOutsideMapBounds(error.what());
      case ErrorCode::GoalOutside: throw nav2_core::GoalOutsideMapBounds(error.what());
      case ErrorCode::StartOccupied: throw nav2_core::StartOccupied(error.what());
      case ErrorCode::GoalOccupied: throw nav2_core::GoalOccupied(error.what());
      case ErrorCode::NoPath: throw nav2_core::NoValidPathCouldBeFound(error.what());
      case ErrorCode::Timeout: throw nav2_core::PlannerTimedOut(error.what());
      case ErrorCode::Cancelled: throw nav2_core::PlannerCancelled(error.what());
      default: throw nav2_core::PlannerException(error.what());
    }
  }
}
}  // namespace transport_orthogonal_planner

PLUGINLIB_EXPORT_CLASS(transport_orthogonal_planner::OrthogonalPlanner, nav2_core::GlobalPlanner)
