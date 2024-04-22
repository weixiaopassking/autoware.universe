// Copyright 2023 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_AVOIDANCE_BY_LANE_CHANGE_MODULE__MANAGER_HPP_
#define BEHAVIOR_PATH_AVOIDANCE_BY_LANE_CHANGE_MODULE__MANAGER_HPP_

#include "behavior_path_avoidance_by_lane_change_module/data_structs.hpp"
#include "behavior_path_avoidance_by_lane_change_module/interface.hpp"
#include "behavior_path_lane_change_module/manager.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware
{
namespace behavior_path_planner
{
using ::route_handler::Direction;
using ::behavior_path_planner::AvoidancePlanningData;
using ::behavior_path_planner::LaneChangeModuleManager;
using ::behavior_path_planner::LaneChangeModuleType;
using ::behavior_path_planner::SceneModuleInterface;
using ::behavior_path_planner::ObjectParameter;
using ::behavior_path_planner::SceneModuleManagerInterface;

class AvoidanceByLaneChangeModuleManager : public LaneChangeModuleManager
{
public:
  AvoidanceByLaneChangeModuleManager()
  : LaneChangeModuleManager(
      "avoidance_by_lane_change", route_handler::Direction::NONE,
      LaneChangeModuleType::AVOIDANCE_BY_LANE_CHANGE)
  {
  }

  void init(rclcpp::Node * node) override;

  std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() override;

private:
  std::shared_ptr<AvoidanceByLCParameters> avoidance_parameters_;
};
}  // namespace behavior_path_planner
}  // namespace autoware

#endif  // BEHAVIOR_PATH_AVOIDANCE_BY_LANE_CHANGE_MODULE__MANAGER_HPP_
