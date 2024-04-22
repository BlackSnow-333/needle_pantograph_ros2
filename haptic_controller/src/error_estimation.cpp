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

#include <memory>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


class ErrorEstimation : public rclcpp::Node
{
public:
  ErrorEstimation()
  : Node("error_estimation"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    int_marker_sub_ = this->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
      "/trajectory_marker/feedback", 10,
      std::bind(&ErrorEstimation::marker_callback, this, std::placeholders::_1));

    // Create error publisher
    error_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/angle_error", 10);

    // Timer to trigger error calculation and logging every second (only for testing)
    timer_ =
      this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&ErrorEstimation::calculate_and_log_error, this));

    RCLCPP_INFO(this->get_logger(), "Error Estimation Node has been started.");
  }

private:
  void marker_callback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr msg)
  {
    if (msg->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
      // Extract position and orientation information from the feedback message
      // The marker is an arrow that starts at PI, has an arbitrary length
      // and its orientaition its defined by the user via an interactive marker
      auto arrow_position = msg->pose.position;
      auto arrow_orientation = msg->pose.orientation;

      // Convert arrow position and orientation to Eigen vectors
      arrow_start << arrow_position.x, arrow_position.y, arrow_position.z;
      Eigen::Quaterniond arrow_quaternion(arrow_orientation.w, arrow_orientation.x,
        arrow_orientation.y, arrow_orientation.z);
      arrow_end = arrow_start + arrow_quaternion * Eigen::Vector3d(1.0, 0.0, 0.0);
    }
  }

  void calculate_and_log_error()
  {
    try {
      auto transform_stamped = tf_buffer_.lookupTransform(
        "world", "needle_interaction_link",
        tf2::TimePointZero);

      // Extract needle interaction point (PU) position
      auto PU = transform_stamped.transform.translation;
      Eigen::Vector3d PU_vec(PU.x, PU.y, PU.z);

      // Vector from insertion point (PI) to arrow tip
      // Vectors are defined in PI frame
      Eigen::Vector3d marker_vec = arrow_end - arrow_start;

      // Vector from PI to PU
      Eigen::Vector3d PIU_vec = PU_vec - arrow_start;

      // Calculate the angle between the arrow vector
      // and the vector from the fixed point to the third point
      double angle_error = std::acos(marker_vec.normalized().dot(PIU_vec.normalized()));

      // RCLCPP_INFO(
      //   this->get_logger(), "Angular error between needle and trajectory: %.2f", angle_error);

      // Publish angle error
      auto msg = std_msgs::msg::Float64();
      msg.data = angle_error;
      error_publisher_->publish(msg);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        this->get_logger(), "Could not transform from 'world' to 'needle_interaction_link': %s",
        ex.what());
    }
  }

  Eigen::Vector3d last_marker_position_;
  Eigen::Vector3d arrow_start;
  Eigen::Vector3d arrow_end;
  rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr
    int_marker_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_publisher_;

  /// Coords of insertion point according to CAD
  double PI_x = 0.0425;
  double PI_y = 0.16056;
  double PI_z = 0.09;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ErrorEstimation>());
  rclcpp::shutdown();
  return 0;
}
