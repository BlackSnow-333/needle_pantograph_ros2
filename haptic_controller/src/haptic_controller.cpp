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

controller_interface::CallbackReturn HapticController::on_init()
{
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

  // Create subscription to marker topic
  marker_subscriber_ = \
    get_node()->create_subscription<visualization_msgs::msg::Marker>(
    "/visualization_marker", 10,
    std::bind(&HapticController::marker_callback, this, std::placeholders::_1));

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
  command_interfaces_config.names.push_back(
    pantograph_joint_names_[0] + "/" + HW_IF_EFFORT);
  command_interfaces_config.names.push_back(
    pantograph_joint_names_[4] + "/" + HW_IF_EFFORT);


  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration HapticController::state_interface_configuration()
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
  if (param_listener_->is_old(params_)) {
    if (read_parameters() != controller_interface::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to read parameters during update");
      return controller_interface::return_type::ERROR;
    }
  }

  for (const auto & state_interface : state_interfaces_) {
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

  Eigen::Vector2d q = Eigen::Vector2d(last_pos_a1, last_pos_a5);
  Eigen::Vector2d q_dot = Eigen::Vector2d(last_vel_a1, last_vel_a5);

  // Compute the pantograph jacobian:
  Eigen::Matrix2d J_panto = pantograph_model_.jacobian(q);

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
  if (!angle_error_msg || !(*angle_error_msg)) {
    last_error_ = 0.0;
    return controller_interface::return_type::OK;
  }

  last_error_ = (*angle_error_msg)->data;

  // Print info in terminal (for testing only)
  // RCLCPP_INFO(get_node()->get_logger(), "angle error : %2f ", last_error_);

  // ==========================================================
  // TODO compute joint torques according to force control law
  // ==========================================================
  Eigen::Vector3d v_needle, v_target, v_proj;

  // Pantograph end effector position :
  Eigen::Vector2d p = pantograph_model_.fk(q);

  // Pantograph end effector velocities
  Eigen::Vector2d p_dot = J_panto * q_dot;

  // System end effector position :
  Eigen::Vector3d PU = pantograph_model_.fk_system(q);

  // Vector from PI to PU:
  v_needle[0] = PU[0] - pantograph_model_.PI_x;
  v_needle[1] = PU[1] - pantograph_model_.PI_y;
  v_needle[2] = PU[1] - pantograph_model_.PI_z;

  // Vector from PI to Ptarget
  v_target[0] = marker_pos[0] - pantograph_model_.PI_x;
  v_target[1] = marker_pos[1] - pantograph_model_.PI_y;
  v_target[2] = marker_pos[2] - pantograph_model_.PI_z;

  // Projection of v_needle in trajectory path
  v_proj = -v_needle.norm() * std::cos(last_error_) * v_target;


  // Force unit vector
  Eigen::Vector3d v_force;
  v_force = (v_proj - v_needle) / (v_proj.norm() - v_needle.norm());


  // Force felt by the user
  // Proportional gain adjust according to desired guiding force
  double Kp = 5;
  Eigen::Vector3d F_u = Kp * last_error_ * v_force;

  // Force that the pantograph needs to generate to create F_u
  // Force magnitude :
  double alpha = 45; // replace by alpha calculation alpha = f(F_u)
  double norm_F_mech = pantograph_model_.get_panto_force(PU, F_u.norm(), alpha);

  // F_mech unit vector
  Eigen::Vector2d v_mech;
  v_mech << std::cos(alpha), std::sin(alpha);
  Eigen::Vector2d F_mech = norm_F_mech * v_mech;

  // Computation of the active joint torques
  auto tau = J_panto.transpose() * F_mech;

  // ==========================================================
  // TODO write commands to HW
  // ==========================================================

  // write mimics to HW
  command_interfaces_[0].set_value(tau(0));
  command_interfaces_[4].set_value(tau(1));

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

void haptic_controller::HapticController::marker_callback(
  const visualization_msgs::msg::Marker::SharedPtr msg)
{
  if (msg->type == visualization_msgs::msg::Marker::LINE_STRIP && !msg->points.empty()) {
    auto points = msg->points;
    auto target_point = points[1];
    marker_pos << target_point.x, target_point.y, target_point.z;
  } else {
    marker_pos << 0, 0, 0;
  }
}

}  // namespace haptic_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  haptic_controller::HapticController,
  controller_interface::ControllerInterface)

// PLUGINLIB_EXPORT_CLASS(haptic_controller::HapticController, controller_interface::ControllerInterface)
