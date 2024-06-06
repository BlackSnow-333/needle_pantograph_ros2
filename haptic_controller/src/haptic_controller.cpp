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
// #include "pantograph_mimick_controller/src/pantograph_controller_utils.hpp"
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

  // Initialization for motor speed calculation :
  last_pos_a1_ = 0.0;
  last_pos_a5_ = 0.0;
  current_pos_a1_ = 0.0;
  current_pos_a5_ = 0.0;

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
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
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

  // Get the previous (active) joint torques
  double torque_a1 = get_value(name_if_value_mapping_, pantograph_joint_names_[0], HW_IF_EFFORT);
  double torque_a5 = get_value(name_if_value_mapping_, pantograph_joint_names_[4], HW_IF_EFFORT);

  // ==========================================================
  // Compute motor rotation speeds using numerical derivation
  // ==========================================================
  // Step size :
  double dt = period.seconds();

  // Get the current joint positions
  double current_pos_a1_ =
    get_value(name_if_value_mapping_, pantograph_joint_names_[0], HW_IF_POSITION);
  double current_pos_a5_ =
    get_value(name_if_value_mapping_, pantograph_joint_names_[4], HW_IF_POSITION);

  double current_vel_a1 = (current_pos_a1_ - last_pos_a1_) / dt;
  double current_vel_a5 = (current_pos_a5_ - last_pos_a5_) / dt;

  // Save positions for next calculation
  if (current_pos_a1_ != last_pos_a1_) {
    last_pos_a1_ = current_pos_a1_;
  }
  if (current_pos_a5_ != last_pos_a5_) {
    last_pos_a5_ = current_pos_a5_;
  }

  // Check if positions are close to limit
  // Pantograph home position : Q = [q1, q2] = [3.4025, -0.1901]

  // Save positions and velocities to state vectors
  Eigen::Vector2d q = Eigen::Vector2d(current_pos_a1_, current_pos_a5_);
  Eigen::Vector2d q_dot = Eigen::Vector2d(current_vel_a1, current_vel_a5);

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
  // last_error_ = std::abs(last_error_);  // Check if needed

  // Print info in terminal (for testing only)
  // RCLCPP_INFO(get_node()->get_logger(), "angle error : %2f ", last_error_);

  // ==========================================================
  // Compute joint torques according to force control law
  // ==========================================================
  Eigen::Vector3d v_needle, v_target, v_proj;

  // Pantograph end effector position :
  Eigen::Vector2d P3 = pantograph_model_.fk(q);

  // Compute the pantograph jacobian:
  Eigen::Matrix2d J_panto = pantograph_model_.jacobian(q);

  // Pantograph end effector velocities
  Eigen::Vector2d P3_dot = J_panto * q_dot;

  // System end effector position :
  Eigen::Vector3d PU = pantograph_model_.fk_system(q);

  // Pantograph insertion point
  Eigen::Vector3d PI;
  PI << pantograph_model_.PI_x, pantograph_model_.PI_y, pantograph_model_.PI_z;

  // Needle vector (PI to PU) :
  v_needle = PU - PI;

  // Needle trajectory vector (PI to Ptarget) :
  v_target = marker_pos_ - PI;

  // Projection of Needle vector in trajectory path
  v_proj = -v_needle.norm() * std::cos(last_error_) * v_target.normalized();

  // F_u :Force felt by the user
  // F_u Force unit vector
  Eigen::Vector3d v_u;
  v_u = v_proj - v_needle;

  // Desired guiding force is proportional to error
  double F_guide = kp * last_error_;

  // F_guide should not exceed 5 N
  if (std::abs(F_guide) > 5) {
    F_guide = 5;
  }

  // F_u : Force felt by the user at PU
  Eigen::Vector3d F_u = F_guide * v_u.normalized();

  // F_mech : Force that the pantograph needs to apply to create F_u
  // Compute intersection point between (v_target,v_needle) plane and (x0,y0) plane
  Eigen::Vector3d z_0 = Eigen::Vector3d(0, 0, 1);
  double dot1 = z_0.dot(v_target.normalized());
  double t_int = 0;
  // Check if dot1 is not null
  if (dot1 != 0) {
    t_int = -z_0.dot(PI) / dot1;
  } else {
    t_int = 0;
  }

  // Coords of the intersection point
  Eigen::Vector3d P_int = PI + t_int * v_target.normalized();

  // Compute F_mech unit direction vector
  Eigen::Vector2d v_mech = (P_int.head(2) - P3).normalized();

  // Compute alpha angle
  double alpha = std::atan2(v_mech[1], v_mech[0]);

  // Compute F_mech magnitude
  double norm_F_mech = pantograph_model_.get_panto_force(PU, F_u.norm(), alpha);

  // Force at the pantograph end effector :
  // kd_tip : Damping coefficient
  // double kd = 0.15 * kp;
  Eigen::Vector2d F_mech = norm_F_mech * v_mech - kd_tip * P3_dot;

  // Computation of the active joint torques
  Eigen::Vector2d tau = J_panto.transpose() * F_mech;

  // ==========================================================
  // Write commands to HW
  // ==========================================================
  // Limit the torque in the joints for security purposes
  double limit = 0.25;  // in Nm

  if (abs(tau(0)) > limit) {
    RCLCPP_INFO(
      get_node()->get_logger(), "Torque command tau_0 too high ! ");

    bool sign_tau_0 = std::signbit(tau(0));

    if (sign_tau_0) {
      tau(0) = limit;
    } else {
      tau(0) = -limit;
    }
  }

  if (abs(tau(1)) > limit) {
    RCLCPP_INFO(
      get_node()->get_logger(), "Torque command tau_1 too high ! ");

    bool sign_tau_1 = std::signbit(tau(1));

    if (sign_tau_1) {
      tau(1) = limit;
    } else {
      tau(1) = -limit;
    }
  }

  // Info for debug
  RCLCPP_INFO(
    get_node()->get_logger(),
    "Angle error : %f, F_u (in N) : %.2f, F_mech (in N) : %.2f, Tau (in Nm): %.2f, %.2f",
    last_error_, F_u.norm(), F_mech.norm(), tau(0), tau(1));

  command_interfaces_[0].set_value(tau(0));
  command_interfaces_[1].set_value(tau(1));
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

  // Update kp gain :
  kp = params_.model_parameters.kp;
  // Update kd_tip gain :
  kd_tip = params_.model_parameters.kd_tip;
  // Update ratio for dampening coef:
  ratio = params_.model_parameters.ratio;
  return controller_interface::CallbackReturn::SUCCESS;
}

void haptic_controller::HapticController::marker_callback(
  const visualization_msgs::msg::Marker::SharedPtr msg)
{
  if (msg->type == visualization_msgs::msg::Marker::LINE_STRIP && !msg->points.empty()) {
    auto points = msg->points;
    auto target_point = points[1];
    marker_pos_ << target_point.x, target_point.y, target_point.z;
  } else {
    // If msg is empty default target points to origin
    marker_pos_ << 0.0, 0.0, 0.0;
  }
}

}  // namespace haptic_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  haptic_controller::HapticController,
  controller_interface::ControllerInterface)

// PLUGINLIB_EXPORT_CLASS(haptic_controller::HapticController, controller_interface::ControllerInterface)
