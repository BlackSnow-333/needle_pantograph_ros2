// Copyright 2023, ICube Laboratory, University of Strasbourg
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

#ifndef PANTOGRAPH_MIMICK_CONTROLLER__PANTOGRAPH_MOCK_MOTORS_CONTROLLER_HPP_
#define PANTOGRAPH_MIMICK_CONTROLLER__PANTOGRAPH_MOCK_MOTORS_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "pantograph_mimick_controller/visibility_control.h"

namespace pantograph_mimick_controller
{

/**
 * \brief Mimic controller for a pantograph. Used for visualization.
 */
class PantographMockMotorsController : public controller_interface::ControllerInterface
{
public:
  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  PantographMockMotorsController();

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  ~PantographMockMotorsController() = default;

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  PANTOGRAPH_MIMICK_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<std::string> joint_names_;
  std::string interface_name_;

  std::vector<std::string> command_interface_types_;

protected:
  std::unordered_map<std::string, std::unordered_map<std::string, double>> name_if_value_mapping_;
  std::vector<std::string> pantograph_joint_names_;
};

}  // namespace pantograph_mimick_controller

#endif  // PANTOGRAPH_MIMICK_CONTROLLER__PANTOGRAPH_MOCK_MOTORS_CONTROLLER_HPP_
