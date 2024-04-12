// Copyright 2020 Tier IV, Inc.
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

#include "node.hpp"

#include <behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <motion_utils/trajectory/path_with_lane_id.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <motion_velocity_smoother/smoother/analytical_jerk_constrained_smoother/analytical_jerk_constrained_smoother.hpp>
#include <tier4_autoware_utils/ros/wait_for_param.hpp>
#include <tier4_autoware_utils/transform/transforms.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_routing/Route.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <functional>
#include <memory>
#include <vector>

namespace
{
rclcpp::SubscriptionOptions create_subscription_options(rclcpp::Node * node_ptr)
{
  rclcpp::CallbackGroup::SharedPtr callback_group =
    node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group;

  return sub_opt;
}
}  // namespace

namespace autoware::behavior_velocity_planner
{
using ::behavior_velocity_planner::TrafficSignalStamped;

namespace
{

autoware_auto_planning_msgs::msg::Path to_path(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path_with_id)
{
  autoware_auto_planning_msgs::msg::Path path;
  for (const auto & path_point : path_with_id.points) {
    path.points.push_back(path_point.point);
  }
  return path;
}
}  // namespace

BehaviorVelocityPlannerNode::BehaviorVelocityPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("behavior_velocity_planner_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  planner_data_(*this)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Trigger Subscriber
  trigger_sub_path_with_lane_id_ =
    this->create_subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>(
      "~/input/path_with_lane_id", 1, std::bind(&BehaviorVelocityPlannerNode::on_trigger, this, _1),
      create_subscription_options(this));

  // Subscribers
  sub_predicted_objects_ =
    this->create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
      "~/input/dynamic_objects", 1,
      std::bind(&BehaviorVelocityPlannerNode::on_predicted_objects, this, _1),
      create_subscription_options(this));
  sub_no_ground_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/no_ground_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&BehaviorVelocityPlannerNode::on_no_ground_point_cloud, this, _1),
    create_subscription_options(this));
  sub_vehicle_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "~/input/vehicle_odometry", 1, std::bind(&BehaviorVelocityPlannerNode::on_odometry, this, _1),
    create_subscription_options(this));
  sub_acceleration_ = this->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "~/input/accel", 1, std::bind(&BehaviorVelocityPlannerNode::on_acceleration, this, _1),
    create_subscription_options(this));
  sub_lanelet_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/vector_map", rclcpp::QoS(10).transient_local(),
    std::bind(&BehaviorVelocityPlannerNode::on_lanelet_map, this, _1),
    create_subscription_options(this));
  sub_traffic_signals_ =
    this->create_subscription<autoware_perception_msgs::msg::TrafficSignalArray>(
      "~/input/traffic_signals", 1,
      std::bind(&BehaviorVelocityPlannerNode::on_traffic_signals, this, _1),
      create_subscription_options(this));
  sub_external_velocity_limit_ = this->create_subscription<VelocityLimit>(
    "~/input/external_velocity_limit_mps", rclcpp::QoS{1}.transient_local(),
    std::bind(&BehaviorVelocityPlannerNode::on_external_velocity_limit, this, _1));
  sub_virtual_traffic_light_states_ =
    this->create_subscription<tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>(
      "~/input/virtual_traffic_light_states", 1,
      std::bind(&BehaviorVelocityPlannerNode::on_virtual_traffic_light_states, this, _1),
      create_subscription_options(this));
  sub_occupancy_grid_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "~/input/occupancy_grid", 1,
    std::bind(&BehaviorVelocityPlannerNode::on_occupancy_grid, this, _1),
    create_subscription_options(this));

  srv_load_plugin_ = create_service<LoadPlugin>(
    "~/service/load_plugin", std::bind(&BehaviorVelocityPlannerNode::on_load_plugin, this, _1, _2));
  srv_unload_plugin_ = create_service<UnloadPlugin>(
    "~/service/unload_plugin",
    std::bind(&BehaviorVelocityPlannerNode::on_unload_plugin, this, _1, _2));

  // set velocity smoother param
  on_param();

  // Publishers
  path_pub_ = this->create_publisher<autoware_auto_planning_msgs::msg::Path>("~/output/path", 1);
  stop_reason_diag_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("~/output/stop_reason", 1);
  debug_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/path", 1);

  // Parameters
  forward_path_length_ = declare_parameter<double>("forward_path_length");
  backward_path_length_ = declare_parameter<double>("backward_path_length");
  behavior_output_path_interval_ = declare_parameter<double>("behavior_output_path_interval");
  planner_data_.stop_line_extend_length = declare_parameter<double>("stop_line_extend_length");

  // nearest search
  planner_data_.ego_nearest_dist_threshold =
    declare_parameter<double>("ego_nearest_dist_threshold");
  planner_data_.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");

  // is simulation or not
  planner_data_.is_simulation = declare_parameter<bool>("is_simulation");

  // Initialize PlannerManager
  for (const auto & name : declare_parameter<std::vector<std::string>>("launch_modules")) {
    // workaround: Since ROS 2 can't get empty list, launcher set [''] on the parameter.
    if (name.empty()) {
      break;
    }
    planner_manager_.launch_scene_plugin(*this, name);
  }

  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);
  published_time_publisher_ = std::make_unique<tier4_autoware_utils::PublishedTimePublisher>(this);
}

void BehaviorVelocityPlannerNode::on_load_plugin(
  const LoadPlugin::Request::SharedPtr request,
  [[maybe_unused]] const LoadPlugin::Response::SharedPtr response)
{
  std::unique_lock<std::mutex> lk(mutex_);
  planner_manager_.launch_scene_plugin(*this, request->plugin_name);
}

void BehaviorVelocityPlannerNode::on_unload_plugin(
  const UnloadPlugin::Request::SharedPtr request,
  [[maybe_unused]] const UnloadPlugin::Response::SharedPtr response)
{
  std::unique_lock<std::mutex> lk(mutex_);
  planner_manager_.remove_scene_plugin(*this, request->plugin_name);
}

// NOTE: argument planner_data must not be referenced for multithreading
bool BehaviorVelocityPlannerNode::is_data_ready(
  const PlannerData & planner_data, rclcpp::Clock clock) const
{
  const auto & d = planner_data;

  // from callbacks
  if (!d.current_odometry) {
    RCLCPP_INFO_THROTTLE(get_logger(), clock, 3000, "Waiting for current odometry");
    return false;
  }

  if (!d.current_velocity) {
    RCLCPP_INFO_THROTTLE(get_logger(), clock, 3000, "Waiting for current velocity");
    return false;
  }
  if (!d.current_acceleration) {
    RCLCPP_INFO_THROTTLE(get_logger(), clock, 3000, "Waiting for current acceleration");
    return false;
  }
  if (!d.predicted_objects) {
    RCLCPP_INFO_THROTTLE(get_logger(), clock, 3000, "Waiting for predicted_objects");
    return false;
  }
  if (!d.no_ground_pointcloud) {
    RCLCPP_INFO_THROTTLE(get_logger(), clock, 3000, "Waiting for pointcloud");
    return false;
  }
  if (!map_ptr_) {
    RCLCPP_INFO_THROTTLE(get_logger(), clock, 3000, "Waiting for the initialization of map");
    return false;
  }
  if (!d.velocity_smoother_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), clock, 3000, "Waiting for the initialization of velocity smoother");
    return false;
  }
  if (!d.occupancy_grid) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), clock, 3000, "Waiting for the initialization of occupancy grid map");
    return false;
  }
  return true;
}

void BehaviorVelocityPlannerNode::on_occupancy_grid(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.occupancy_grid = msg;
}

void BehaviorVelocityPlannerNode::on_predicted_objects(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.predicted_objects = msg;
}

void BehaviorVelocityPlannerNode::on_no_ground_point_cloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      "map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), "no transform found for no_ground_pointcloud: %s", e.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::fromROSMsg(*msg, pc);

  Eigen::Affine3f affine = tf2::transformToEigen(transform.transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZ>);
  if (!pc.empty()) {
    tier4_autoware_utils::transformPointCloud(pc, *pc_transformed, affine);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    planner_data_.no_ground_pointcloud = pc_transformed;
  }
}

void BehaviorVelocityPlannerNode::on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto current_odometry = std::make_shared<geometry_msgs::msg::PoseStamped>();
  current_odometry->header = msg->header;
  current_odometry->pose = msg->pose.pose;
  planner_data_.current_odometry = current_odometry;

  auto current_velocity = std::make_shared<geometry_msgs::msg::TwistStamped>();
  current_velocity->header = msg->header;
  current_velocity->twist = msg->twist.twist;
  planner_data_.current_velocity = current_velocity;

  // Add velocity to buffer
  planner_data_.velocity_buffer.push_front(*current_velocity);
  const rclcpp::Time now = this->now();
  while (!planner_data_.velocity_buffer.empty()) {
    // Check oldest data time
    const auto & s = planner_data_.velocity_buffer.back().header.stamp;
    const auto time_diff =
      now >= s ? now - s : rclcpp::Duration(0, 0);  // Note: negative time throws an exception.

    // Finish when oldest data is newer than threshold
    if (time_diff.seconds() <= PlannerData::velocity_buffer_time_sec) {
      break;
    }

    // Remove old data
    planner_data_.velocity_buffer.pop_back();
  }
}

void BehaviorVelocityPlannerNode::on_acceleration(
  const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.current_acceleration = msg;
}

void BehaviorVelocityPlannerNode::on_param()
{
  // Note(VRichardJP): mutex lock is not necessary as on_param is only called once in the
  // constructed. It would be required if it was a callback. std::lock_guard<std::mutex>
  // lock(mutex_);
  planner_data_.velocity_smoother_ =
    std::make_unique<motion_velocity_smoother::AnalyticalJerkConstrainedSmoother>(*this);
}

void BehaviorVelocityPlannerNode::on_lanelet_map(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  map_ptr_ = msg;
  has_received_map_ = true;
}

void BehaviorVelocityPlannerNode::on_traffic_signals(
  const autoware_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // clear previous observation
  planner_data_.traffic_light_id_map_raw_.clear();
  const auto traffic_light_id_map_last_observed_old =
    planner_data_.traffic_light_id_map_last_observed_;
  planner_data_.traffic_light_id_map_last_observed_.clear();
  for (const auto & signal : msg->signals) {
    TrafficSignalStamped traffic_signal;
    traffic_signal.stamp = msg->stamp;
    traffic_signal.signal = signal;
    planner_data_.traffic_light_id_map_raw_[signal.traffic_signal_id] = traffic_signal;
    const bool is_unknown_observation =
      std::any_of(signal.elements.begin(), signal.elements.end(), [](const auto & element) {
        return element.color == autoware_perception_msgs::msg::TrafficSignalElement::UNKNOWN;
      });
    // if the observation is UNKNOWN and past observation is available, only update the timestamp
    // and keep the body of the info
    const auto old_data = traffic_light_id_map_last_observed_old.find(signal.traffic_signal_id);
    if (is_unknown_observation && old_data != traffic_light_id_map_last_observed_old.end()) {
      // copy last observation
      planner_data_.traffic_light_id_map_last_observed_[signal.traffic_signal_id] =
        old_data->second;
      // update timestamp
      planner_data_.traffic_light_id_map_last_observed_[signal.traffic_signal_id].stamp =
        msg->stamp;
    } else {
      planner_data_.traffic_light_id_map_last_observed_[signal.traffic_signal_id] = traffic_signal;
    }
  }
}

void BehaviorVelocityPlannerNode::on_external_velocity_limit(const VelocityLimit::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.external_velocity_limit = *msg;
}

void BehaviorVelocityPlannerNode::on_virtual_traffic_light_states(
  const tier4_v2x_msgs::msg::VirtualTrafficLightStateArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  planner_data_.virtual_traffic_light_states = msg;
}

void BehaviorVelocityPlannerNode::on_trigger(
  const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg)
{
  std::unique_lock<std::mutex> lk(mutex_);

  if (!is_data_ready(planner_data_, *get_clock())) {
    return;
  }

  // Load map and check route handler
  if (has_received_map_) {
    planner_data_.route_handler_ = std::make_shared<route_handler::RouteHandler>(*map_ptr_);
    has_received_map_ = false;
  }
  if (!planner_data_.route_handler_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000, "Waiting for the initialization of route_handler");
    return;
  }

  if (input_path_msg->points.empty()) {
    return;
  }

  const autoware_auto_planning_msgs::msg::Path output_path_msg =
    generate_path(input_path_msg, planner_data_);

  lk.unlock();

  path_pub_->publish(output_path_msg);
  published_time_publisher_->publish_if_subscribed(path_pub_, output_path_msg.header.stamp);
  stop_reason_diag_pub_->publish(planner_manager_.get_stop_reason_diag());

  if (debug_viz_pub_->get_subscription_count() > 0) {
    publish_debug_marker(output_path_msg);
  }
}

autoware_auto_planning_msgs::msg::Path BehaviorVelocityPlannerNode::generate_path(
  const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg,
  const PlannerData & planner_data)
{
  autoware_auto_planning_msgs::msg::Path output_path_msg;

  // TODO(someone): support backward path
  const auto is_driving_forward = motion_utils::isDrivingForward(input_path_msg->points);
  is_driving_forward_ = is_driving_forward ? is_driving_forward.value() : is_driving_forward_;
  if (!is_driving_forward_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Backward path is NOT supported. just converting path_with_lane_id to path");
    output_path_msg = to_path(*input_path_msg);
    output_path_msg.header.frame_id = "map";
    output_path_msg.header.stamp = this->now();
    output_path_msg.left_bound = input_path_msg->left_bound;
    output_path_msg.right_bound = input_path_msg->right_bound;
    return output_path_msg;
  }

  // Plan path velocity
  const auto velocity_planned_path = planner_manager_.plan_path_velocity(
    std::make_shared<const PlannerData>(planner_data), *input_path_msg);

  // screening
  const auto filtered_path =
    ::behavior_velocity_planner::filterLitterPathPoint(to_path(velocity_planned_path));

  // interpolation
  const auto interpolated_path_msg = ::behavior_velocity_planner::interpolatePath(
    filtered_path, forward_path_length_, behavior_output_path_interval_);

  // check stop point
  output_path_msg = ::behavior_velocity_planner::filterStopPathPoint(interpolated_path_msg);

  output_path_msg.header.frame_id = "map";
  output_path_msg.header.stamp = this->now();

  // TODO(someone): This must be updated in each scene module, but copy from input message for now.
  output_path_msg.left_bound = input_path_msg->left_bound;
  output_path_msg.right_bound = input_path_msg->right_bound;

  return output_path_msg;
}

void BehaviorVelocityPlannerNode::publish_debug_marker(
  const autoware_auto_planning_msgs::msg::Path & path)
{
  visualization_msgs::msg::MarkerArray output_msg;
  for (size_t i = 0; i < path.points.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.pose = path.points.at(i).pose;
    marker.scale.y = marker.scale.z = 0.05;
    marker.scale.x = 0.25;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    output_msg.markers.push_back(marker);
  }
  debug_viz_pub_->publish(output_msg);
}
}  // namespace autoware::behavior_velocity_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::behavior_velocity_planner::BehaviorVelocityPlannerNode)
