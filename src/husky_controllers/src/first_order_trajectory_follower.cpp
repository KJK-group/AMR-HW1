#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <utility>
#include <string>
#include <algorithm>
#include "husky_controllers/utility.h"
#include "husky_controllers/Error.h"

#define POS_TOLERANCE 0.2

using namespace std;

// target slope
auto trajectory_slope = 0.f;
// controller gains
auto K_rho = 0.5f;
auto K_alpha = 0.5f;
auto K_beta = 0.f;

ros::Publisher pub_control;
ros::Publisher pub_error;
ros::Subscriber sub_odom;

auto first_order_trajectory(float x) -> float {
    return trajectory_slope * x;
}

auto odometry_filtered_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void {
    // vehicle 2D position
    auto curr_x = msg->pose.pose.position.x;
    auto curr_y = msg->pose.pose.position.y;

    // desired position
    auto desired_y = first_order_trajectory(curr_x);
    // error to desired y coordinated
    auto diff_y = abs(abs(curr_y) - abs(desired_y));
    ROS_WARN("curr_x: %.5f", curr_y);
    ROS_WARN("curr_y: %.5f", curr_y);
    ROS_WARN("desired_y: %.5f", desired_y);
    ROS_WARN("diff_y: %.5f", diff_y);
    ROS_WARN("slope: %.5f", trajectory_slope);
    // error message
    auto error_msg = husky_controllers::Error();
    error_msg.error = diff_y;
    pub_error.publish(error_msg);

    // vehicle yaw
    auto theta = tf::getYaw(msg->pose.pose.orientation);
    // desired heading
    auto beta = -atan(trajectory_slope);
    // heading error
    auto alpha = -beta - theta;
    // correct yaw in range [-pi, pi]
    if (alpha > M_PI) {
        alpha -= 2*M_PI;
    }
    else if (alpha < -M_PI) {
        alpha += 2*M_PI;
    }

    // constant position error as we want to move at a constant speed
    auto rho = 1;

    // control message for cmd_vel
    auto command = geometry_msgs::Twist();

    // controller outputs velocity v and angular velocity omega
    auto v = K_rho * rho;
    auto omega = K_alpha * alpha + K_beta * beta;

    // publish control outputs until the husky is within the tolerance
    if (rho > POS_TOLERANCE) {
        command.linear.x = v;
        command.angular.z = omega;
    }
    else {
        command.linear.x = 0;
        command.angular.z = 0;
    }

    pub_control.publish(command);

    //debug ROS_INFO prints
    ROS_INFO("pose:");
    ROS_INFO("  x:     %.5f", curr_x);
    ROS_INFO("  y:     %.5f", curr_y);
    ROS_INFO("  theta: %.5f", theta);

    ROS_INFO("errors:");
    ROS_INFO("  rho:   %.5f", rho);
    ROS_INFO("  beta:  %.5f", beta);
    ROS_INFO("  alpha: %.5f", alpha);

    ROS_INFO("control inputs:");
    ROS_INFO("  k_rho:   %.5f", K_rho);
    ROS_INFO("  k_alpha: %.5f", K_alpha);
    ROS_INFO("  k_beta:  %.5f", K_beta);

    ROS_INFO("control outputs:");
    ROS_INFO("  v:     %.5f", v);
    ROS_INFO("  omega: %.5f", omega);
}

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "first_order_trajectory_follower");
    auto nh = ros::NodeHandle("~");

    // control publisher to cmd_vel
    pub_control = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);
    // error (rho) publisher to custom topic
    pub_error = nh.advertise<husky_controllers::Error>("/husky_controllers/position_error", 10);

    // passing command line arguments
    // trajectory slope
    if (argc > 1) trajectory_slope = stof(argv[1]);
    // controller gains
    if (argc > 2) K_rho = stof(argv[2]);
    if (argc > 3) K_alpha = stof(argv[3]);
    if (argc > 4) K_beta = stof(argv[4]);

    // subscriber to odometry filtered data from the husky
    sub_odom = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, odometry_filtered_cb);

    auto loop_rate = ros::Rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}