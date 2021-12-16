#include "../include/bot_controller.h"
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <cmath>

Bot_Controller::Bot_Controller(ros::NodeHandle* nodehandle) :
    m_nh{ *nodehandle },
    m_kv{ 0.1 },
    m_kh{ 0.5 },
    m_parent_frame{ "odom" },
    m_child_frame{ "base_footprint" },
    m_location{ 0,0 },
    m_linear_speed{ 0.3 },
    m_angular_speed{ 0.5 },
    m_roll{ 0 },
    m_pitch{ 0 },
    m_yaw{ 0 }
{
    m_initialize_subscribers();
    m_initialize_publishers();
}


double Bot_Controller::m_normalize_angle_positive(double angle)
{
    const double result = fmod(angle, 2.0 * M_PI);
    if (result < 0) return result + 2.0 * M_PI;
    return result;
}


double Bot_Controller::m_normalize_angle(double angle)
{
    const double result = fmod(angle + M_PI, 2.0 * M_PI);
    if (result <= 0.0) return result + M_PI;
    return result - M_PI;
}

void Bot_Controller::m_initialize_publishers() {
    // ROS_INFO("Initializing Publishers");
    m_velocity_publisher = m_nh.advertise<geometry_msgs::Twist>("explorer/cmd_vel", 100);
    //add more publishers here as needed
}

void Bot_Controller::m_initialize_subscribers() {
    // ROS_INFO("Initializing Subscribers");
    m_pose_subscriber = m_nh.subscribe("odom", 1000, &Bot_Controller::m_pose_callback, this);
    //add more subscribers as needed
}

void Bot_Controller::m_pose_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    m_location.first = msg->pose.pose.position.x;
    m_location.second = msg->pose.pose.position.y;
    m_orientation = msg->pose.pose.orientation;
    // compute_yaw();

    ROS_INFO_STREAM("-------------------------");
    ROS_INFO_STREAM("Pose of the robot: "
        << "[" << m_location.first
        << ","
        << m_location.second << "], [" << m_orientation.x << ","
        << m_orientation.y << ","
        << m_orientation.z << "," << m_orientation.w << "]");
}

void Bot_Controller::m_move(double linear, double angular) {
    geometry_msgs::Twist msg;
    msg.linear.x = linear;
    msg.angular.z = angular;
    m_velocity_publisher.publish(msg);
}

void Bot_Controller::stop() {
    ROS_DEBUG_STREAM("Stopping robot");
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
    m_velocity_publisher.publish(msg);
    //ros::shutdown();
}

double Bot_Controller::compute_expected_final_yaw(bool direction, double angle_to_rotate) {
    double current_yaw_deg = compute_yaw_deg();

    double final_angle{};//final angle after rotating angle_to_rotate
    if (direction)
        final_angle = current_yaw_deg + angle_to_rotate;
    else
        final_angle = current_yaw_deg - angle_to_rotate;
    ROS_INFO_STREAM("Current ANGLE: " << current_yaw_deg);
    ROS_INFO_STREAM("Final ANGLE: " << final_angle);
    // ros::shutdown();

    return final_angle;
}

double Bot_Controller::compute_yaw_deg() {
    // other method to compute yaw from quaternion
    // double siny_cosp = 2 * (m_orientation.w * m_orientation.z + m_orientation.x * m_orientation.y);
    // double cosy_cosp = 1 - 2 * (m_orientation.y * m_orientation.x + m_orientation.z * m_orientation.z);
    // double yaw_rad = std::atan2(siny_cosp, cosy_cosp);
    // yaw_rad = m_normalize_angle_positive(yaw_rad);
    // double current_yaw_deg = yaw_rad * 180.0 / M_PI;
    double roll{};
    double pitch{};
    double yaw_rad{};
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(m_orientation, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw_rad);
    yaw_rad = m_normalize_angle_positive(yaw_rad);
    double current_yaw_deg = yaw_rad * 180.0 / M_PI;
    return current_yaw_deg;
}

void Bot_Controller::rotate(double angle_to_rotate, bool direction, double final_angle) {
    double current_yaw_deg = compute_yaw_deg();

    ROS_INFO_STREAM("Rotate an angle of [" << angle_to_rotate << " degrees]");
    ROS_INFO_STREAM("Current orientation: " << current_yaw_deg);
    ROS_INFO_STREAM("Final orientation: " << final_angle);

    if (direction) {
        if (final_angle >= current_yaw_deg) {
            m_move(0.0, m_angular_speed);
        }
        else {
            //stop the robot
            stop();
        }
    }
    else {
        if (final_angle <= current_yaw_deg) {
            m_move(0.0, -m_angular_speed);
        }
        else {
            //stop the robot
            stop();
        }
    }

}
