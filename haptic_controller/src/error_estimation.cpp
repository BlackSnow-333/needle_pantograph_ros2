#include <memory>
#include <cmath> // for sqrt and pow functions

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry> 

class ErrorEstimation : public rclcpp::Node
{
public:
    ErrorEstimation() : Node("error_estimation"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        marker_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
            "/needle_trajectory", 10, std::bind(&ErrorEstimation::marker_callback, this, std::placeholders::_1));

        // Timer to trigger error calculation and logging every second (only for testing)
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ErrorEstimation::calculate_and_log_error, this));

        RCLCPP_INFO(this->get_logger(), "Error Estimation Node has been started.");
    }

private:

    void marker_callback(const visualization_msgs::msg::Marker::SharedPtr marker_msg)
    {

        if (marker_msg->type == visualization_msgs::msg::Marker::LINE_STRIP)
        {
            // last_marker_position_ = marker_msg->pose.position;
            last_marker_position_ = marker_msg->points[1];
            
            // RCLCPP_INFO(this->get_logger(), "Received marker at position: (%.2f, %.2f, %.2f)",
            //             last_marker_position_.x, last_marker_position_.y, last_marker_position_.z);
            // calculate_and_log_error();
        }
    }

    // void calculate_and_log_error()
    // {
    //     try
    //     {
    //         auto transform_stamped = tf_buffer_.lookupTransform("world", "needle_interaction_link", tf2::TimePointZero);

    //         // Calculate the error
    //         auto error_x = last_marker_position_.x - transform_stamped.transform.translation.x;
    //         auto error_y = last_marker_position_.y - transform_stamped.transform.translation.y;
    //         auto error_z = last_marker_position_.z - transform_stamped.transform.translation.z;
    //         double magnitude_error = sqrt(pow(error_x, 2) + pow(error_y, 2) + pow(error_z, 2));

    //         RCLCPP_INFO(this->get_logger(), "Error Magnitude: %.2f", magnitude_error);
    //     }
    //     catch (tf2::TransformException &ex)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Could not transform from 'world' to 'needle_interaction_link': %s", ex.what());
    //     }
    // }

    void calculate_and_log_error()
    {
        try
        {
            auto transform_stamped = tf_buffer_.lookupTransform("world", "needle_interaction_link", tf2::TimePointZero);

            // Extract needle interaction point (PU) position
            auto PU = transform_stamped.transform.translation;

            // Define vectors from insertion point (PI) to marker point and PU
            // Vectors are defined in PI frame
            Eigen::Vector3d marker_vec(last_marker_position_.x - PI_x, 
                    last_marker_position_.y - PI_y, 
                    last_marker_position_.z - PI_z);

            Eigen::Vector3d PU_vec(PU.x - PI_x, PU.y - PI_y, PU.z - PI_z);

            // Direction vector of the line from insertion point to marker point
            Eigen::Vector3d line_dir = marker_vec.normalized();

            // Point on the line closest to end effector (projection)
            double t = PU_vec.dot(line_dir);
            Eigen::Vector3d closest_point = t * line_dir;

            // Calculate distance between end effector point and closest point on the line
            double magnitude_error = (PU_vec - closest_point).norm();

            RCLCPP_INFO(this->get_logger(), "Distance between needle and trajectory: %.2f", magnitude_error);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not transform from 'world' to 'needle_interaction_link': %s", ex.what());
        }
    }

    geometry_msgs::msg::Point_<std::allocator<void>> last_marker_position_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    /// Coords of insertion point according to CAD
    double PI_x = 0.0425;
    double PI_y = 0.16056;
    double PI_z = 0.09;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ErrorEstimation>());
    rclcpp::shutdown();
    return 0;
}
