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

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "geometry_msgs/msg/point.hpp"

class InteractiveMarkerNode : public rclcpp::Node
{
public:
  InteractiveMarkerNode()
  : Node("interactive_marker_node")
  {
    marker_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
    // interactive_marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("interactive_marker");

    // create an interactive marker server on the topic namespace interactive_marker
    interactive_markers::InteractiveMarkerServer interactive_marker_server_("interactive_marker",
      this);

    // Create the fixed starting point
    // Coords of insertion point according to CAD
    start_point_.x = 0.0425;
    start_point_.y = 0.16056;
    start_point_.z = 0.09;

    // Create an interactive marker for the end point
    createInteractiveMarker("end_point", end_point_, true);

    // Publish the line between start and end points
    publishLine();

    // Set up a timer to update the marker
    timer_ =
      this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&InteractiveMarkerNode::timerCallback, this));
  }

private:
  void createInteractiveMarker(
    const std::string & name, geometry_msgs::msg::Point & point,
    bool movable)
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "world";
    int_marker.header.stamp = this->now();
    int_marker.name = name;
    int_marker.description = name;
    int_marker.pose.position = point;
    int_marker.pose.orientation.w = 1.0;

    visualization_msgs::msg::InteractiveMarkerControl control;
    control.always_visible = true;
    control.orientation.w = 1.0;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;

    if (movable) {
      control.name = "move";
    }

    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    control.markers.push_back(marker);
    control.markers[0].pose.position = point;
    control.markers[0].header.frame_id = "world";
    control.markers[0].header.stamp = this->now();

    int_marker.controls.push_back(control);

    interactive_marker_server_->insert(int_marker);
    interactive_marker_server_->applyChanges();
  }

  void publishLine()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.points.push_back(start_point_);
    marker.points.push_back(end_point_);

    marker.lifetime = rclcpp::Duration(0, 0);     // Set lifetime to infinite

    marker_pub_->publish(marker);
  }

  void timerCallback()
  {
    // Update the line's end point based on the interactive marker
    visualization_msgs::msg::InteractiveMarker int_marker;
    if (interactive_marker_server_->get("end_point", int_marker)) {
      end_point_ = int_marker.pose.position;
      publishLine();
    }
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Point start_point_;
  geometry_msgs::msg::Point end_point_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InteractiveMarkerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
