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
    marker_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
      "/visualization_marker", 10,
      std::bind(&ErrorEstimation::marker_callback, this, std::placeholders::_1));

    // Create error publisher
    error_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/angle_error", 10);

    // Timer to trigger error calculation and logging every second (only for testing)
    timer_ =
      this->create_wall_timer(
      std::chrono::milliseconds(2),
      std::bind(&ErrorEstimation::calculate_and_log_error, this));

    RCLCPP_INFO(this->get_logger(), "Error Estimation Node has been started.");
  }

private:
  void marker_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
  {
    if (msg->points.size() != 2) {
      RCLCPP_ERROR(this->get_logger(), "Received marker does not have exactly 2 points.");
      return;
    }

    if (msg->points.size() == 0) {
      // If msg is empty default points to origin
      p0.x = PI_x;
      p0.y = PI_y;
      p0.z = PI_z;

      p1.x = 0;
      p1.y = 0;
      p1.z = 0;
    } else {
      // Extract the points from the marker
      p0 = msg->points[0];  // Insertion point
      p1 = msg->points[1];  // Target point from marker
    }
  }

  void calculate_and_log_error()
  {
    // calculate and log error
    try {
      auto transform_stamped = tf_buffer_.lookupTransform(
        "world", "needle_interaction_link",
        tf2::TimePointZero);

      // Needle insertion point
      Eigen::Vector3d PI;
      PI << PI_x, PI_y, PI_z;

      // Extract needle interaction point (PU) position
      auto PU = transform_stamped.transform.translation;
      Eigen::Vector3d PU_vec(PU.x, PU.y, PU.z);

      // Vector from insertion point (PI) to arrow tip
      // Vectors are defined in PI frame
      Eigen::Vector3d marker_vec(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);

      // Vector from PI to PU
      Eigen::Vector3d needle_vec = PU_vec - PI;

      // Calculate the angle between the arrow vector
      // and the vector from the fixed point to the third point
      double angle_error = PI_CST - std::acos(
        needle_vec.dot(marker_vec) /
        (needle_vec.norm() * marker_vec.norm()));

      if (isnan(angle_error)) {
        // RCLCPP_INFO(
        //   this->get_logger(), "Unable to calculate error, target point not defined ! ");

        // RCLCPP_INFO(
        //   this->get_logger(), "Angle error set to %.2f ", 0.0);

        // Publish angle error default value
        auto msg = std_msgs::msg::Float64();
        msg.data = 0.0;
        error_publisher_->publish(msg);
      } else {
        // RCLCPP_INFO(
        //   this->get_logger(), "Angular error between needle and trajectory: %.2f", angle_error);

        // Publish angle error
        auto msg = std_msgs::msg::Float64();
        msg.data = angle_error;
        error_publisher_->publish(msg);
      }
    } catch (tf2::TransformException & ex) {
      // RCLCPP_ERROR(
      //   this->get_logger(), "Could not transform from 'world' to 'needle_interaction_link': %s",
      //   ex.what());
    }
  }

  geometry_msgs::msg::Point p0;
  geometry_msgs::msg::Point p1;

  // Coords of insertion point according to CAD
  double PI_x = 0;
  double PI_y = 0.16056;
  double PI_z = 0.09;

  double PI_CST = 3.14159265358979323846;

  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr error_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ErrorEstimation>());
  rclcpp::shutdown();
  return 0;
}
