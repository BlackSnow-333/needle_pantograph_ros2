// Copyright 2024, ICube Laboratory, University of Strasbourg
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

#include "haptic_controller/haptic_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace haptic_controller 
{

const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::HW_IF_EFFORT;

HapticController::HapticController() 
: controller_interface::ControllerInterface(), pantograph_model_()
{
}

controller_interface::CallbackReturn HapticController::on_init() {
  // Implementation of init method
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HapticController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) 
{
  // Implementation of on_configure method
  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  pantograph_joint_names_ = {
    params_.prefix + "panto_a1",
    params_.prefix + "panto_a2",
    params_.prefix + "panto_a3",
    params_.prefix + "panto_a4",
    params_.prefix + "panto_a5",
    params_.prefix + "tool_theta_joint",
    params_.prefix + "tool_phi_joint",
    params_.prefix + "needle_interaction_joint"};

  // Create subscription to error topic (for testing)
  error_subscriber_ = \
    get_node()->create_subscription<std_msgs::msg::Float64>(
    "/angle_error",
    rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64::SharedPtr msg)
    {rt_command_ptr_.writeFromNonRT(msg);});

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
HapticController::command_interface_configuration() const 
{
  // Implementation of command_interface_configuration method
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.push_back(
    pantograph_joint_names_[0] + "/" + HW_IF_POSITION);
  command_interfaces_config.names.push_back(
    pantograph_joint_names_[4] + "/" + HW_IF_POSITION);
  command_interfaces_config.names.push_back(
    pantograph_joint_names_[0] + "/" + HW_IF_VELOCITY);
  command_interfaces_config.names.push_back(
    pantograph_joint_names_[4] + "/" + HW_IF_VELOCITY);

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration HapticController::
state_interface_configuration() 
const 
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::ALL};
}

controller_interface::CallbackReturn HapticController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) 
{
  // Implementation of on_activate method
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  ordered_interfaces;
  if (
    !controller_interface::get_ordered_interfaces(
      command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
    command_interface_types_.size() != ordered_interfaces.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
      command_interface_types_.size(), ordered_interfaces.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = \
    realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HapticController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) 
{
  return controller_interface::CallbackReturn::SUCCESS;
}

double get_value(
  const std::unordered_map<std::string, std::unordered_map<std::string, double>> & map,
  const std::string & name, const std::string & interface_name)
{
  const auto & interfaces_and_values = map.at(name);
  const auto interface_and_value = interfaces_and_values.find(interface_name);
  if (interface_and_value != interfaces_and_values.cend()) {
    return interface_and_value->second;
  } else {
    return kUninitializedValue;
  }
}

controller_interface::return_type HapticController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (param_listener_->is_old(params_))
    {
        if (read_parameters() != controller_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to read parameters during update");
            return controller_interface::return_type::ERROR;
        }
    }

    for (const auto &state_interface : state_interfaces_)
    {
        name_if_value_mapping_[
            state_interface.get_prefix_name()][state_interface.get_interface_name()] =
            state_interface.get_value();
        RCLCPP_DEBUG(
            get_node()->get_logger(), "%s/%s: %f\n", state_interface.get_prefix_name().c_str(),
            state_interface.get_interface_name().c_str(), state_interface.get_value());
    }

    // Get the previous (active) joint positions and velocities
    double last_pos_a1 =
        get_value(name_if_value_mapping_, pantograph_joint_names_[0], HW_IF_POSITION);
    double last_vel_a1 =
        get_value(name_if_value_mapping_, pantograph_joint_names_[0], HW_IF_VELOCITY);
    double last_pos_a5 =
        get_value(name_if_value_mapping_, pantograph_joint_names_[4], HW_IF_POSITION);
    double last_vel_a5 =
        get_value(name_if_value_mapping_, pantograph_joint_names_[4], HW_IF_VELOCITY);

    // Compute the pantograph jacobian:
    Eigen::Vector<double, 2> q;
    q << last_pos_a1, last_pos_a5;

    Eigen::Matrix2d j_panto = pantograph_model_.jacobian(q);

    // Get the previous (active) joint torques
    double torque_a1 = get_value(name_if_value_mapping_, pantograph_joint_names_[0], HW_IF_EFFORT);
    double torque_a5 = get_value(name_if_value_mapping_, pantograph_joint_names_[4], HW_IF_EFFORT);

    // ==========================================================
    // Compute angle error between desired trajectory (int_marker)
    // and current needle orientation
    // ==========================================================

    // Get angle error from rt buffer
    auto angle_error_msg = rt_command_ptr_.readFromRT();
    // no command received yet
    if (!angle_error_msg || !(*angle_error_msg))
    {
      last_error_ = 0.0;
      return controller_interface::return_type::OK;
    }

    last_error_ = (*angle_error_msg)->data;

    // ==========================================================
    // TODO compute joint torques according to force control law
    // ==========================================================

    // tau_1, tau_5 = id_system(dPu);
   
    // Print info in terminal (for testing only)
    RCLCPP_INFO(get_node()->get_logger(), "angle error : %2f ", last_error_);

    // ==========================================================
    // TODO write commands to HW
    // ==========================================================

    // // Integrate state variables
    // double dt = period.seconds();
    // double vel_a1 = last_vel_a1 + last_acc_a1 * dt;
    // double pos_a1 = last_pos_a1 + vel_a1 * dt;

    // double vel_a5 = last_vel_a5 + last_acc_a5 * dt;
    // double pos_a5 = last_pos_a5 + vel_a5 * dt;

    // command_interfaces_[0].set_value(vel_a1);
    // command_interfaces_[1].set_value(vel_a5);
    
    // command_interfaces_[2].set_value(pos_a1);
    // command_interfaces_[3].set_value(pos_a5);
    
    return controller_interface::return_type::OK;

}

controller_interface::CallbackReturn
HapticController::read_parameters()
{
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  // Update pantograph model parameters
  pantograph_model_.set_link_lenghts(
    params_.model_parameters.l_a1,
    params_.model_parameters.l_a2,
    params_.model_parameters.l_a3,
    params_.model_parameters.l_a4,
    params_.model_parameters.l_a5);

  return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace haptic_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  haptic_controller::HapticController,
  controller_interface::ControllerInterface)

// PLUGINLIB_EXPORT_CLASS(haptic_controller::HapticController, controller_interface::ControllerInterface)
