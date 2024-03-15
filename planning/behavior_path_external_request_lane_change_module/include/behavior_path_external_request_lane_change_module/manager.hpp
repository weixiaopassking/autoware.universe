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

#ifndef BEHAVIOR_PATH_EXTERNAL_REQUEST_LANE_CHANGE_MODULE__MANAGER_HPP_
#define BEHAVIOR_PATH_EXTERNAL_REQUEST_LANE_CHANGE_MODULE__MANAGER_HPP_

#include "behavior_path_lane_change_module/manager.hpp"
#include "route_handler/route_handler.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace autoware
{
namespace behavior_path_planner
{
class ExternalRequestLaneChangeRightModuleManager : public LaneChangeModuleManager
{
public:
  ExternalRequestLaneChangeRightModuleManager()
  : LaneChangeModuleManager(
      "external_request_lane_change_right", route_handler::Direction::RIGHT,
      LaneChangeModuleType::EXTERNAL_REQUEST)
  {
  }
  std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() override;
};

class ExternalRequestLaneChangeLeftModuleManager : public LaneChangeModuleManager
{
public:
  ExternalRequestLaneChangeLeftModuleManager()

  : LaneChangeModuleManager(
      "external_request_lane_change_left", route_handler::Direction::LEFT,
      LaneChangeModuleType::EXTERNAL_REQUEST)
  {
  }
  std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() override;
};
}  // namespace behavior_path_planner
}  // namespace autoware

#endif  // BEHAVIOR_PATH_EXTERNAL_REQUEST_LANE_CHANGE_MODULE__MANAGER_HPP_
