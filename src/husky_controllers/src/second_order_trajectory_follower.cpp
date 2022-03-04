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

#define KP 0.5
#define KA 0.5
#define KB 0.01
#define POS_TOLERANCE 0.2

using namespace std;

float a = 0;

auto trajectory_slope(float x) -> float {
    // ax²  second order polynomial
    // 2ax  slope of second order polynomial
    return 2 * a * x;
}

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "second_order_trajectory_follower");

    auto nh = ros::NodeHandle("~");
    auto pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);

    auto loop_rate = ros::Rate(10);

    a = stof(argv[1]);

    auto odom_cb = [&](const nav_msgs::Odometry::ConstPtr& msg) {
        auto qx = msg->pose.pose.orientation.x;
        auto qy = msg->pose.pose.orientation.y;
        auto qz = msg->pose.pose.orientation.z;
        auto qw = msg->pose.pose.orientation.w;

        auto curr_x = msg->pose.pose.position.x;
        auto curr_y = msg->pose.pose.position.y;

        // Vehicle yaw
        auto theta = get<2>(quarternion_to_euler(qx, qy, qz, qw));
        // Desired heading
        auto beta = atan(trajectory_slope(curr_x));
        // Heading error
        auto alpha = beta - theta;

        auto rho = 1;

        auto command = geometry_msgs::Twist();

        auto v = KP * rho;
        auto omega = KA * alpha;// + KB * beta;

        if (rho > POS_TOLERANCE) {
            command.linear.x = v;
            command.angular.z = omega;
        }
        else {
            command.linear.x = 0;
            command.angular.z = 0;
        }
        ROS_INFO("pose:");
        ROS_INFO("  x:     %.5f", curr_x);
        ROS_INFO("  y:     %.5f", curr_y);
        ROS_INFO("  theta: %.5f", theta);

        ROS_INFO("errors:");
        ROS_INFO("  rho:   %.5f", rho);
        ROS_INFO("  beta:  %.5f", beta);
        ROS_INFO("  alpha: %.5f", alpha);

        pub.publish(command);
    };

    auto sub = nh.subscribe<nav_msgs::Odometry>("/husky_velocity_controller/odom", 10, odom_cb);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}