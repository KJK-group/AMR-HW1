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

#include "utility.h"

#define KP 0.2
#define KA 0.5
#define KB 0.01
#define POS_TOLERANCE 0.2

using namespace std;

auto trajectory_slope = 0;

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, __BASE_FILE__);

    auto euler = quarternion_to_euler(4.1, 6.2, 7.4, 2.9);
    ROS_WARN("x: %.5f\ty: %.5f\tz: %.5f", get<0>(euler), get<1>(euler), get<2>(euler));

    auto nh = ros::NodeHandle();
    auto pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);

    auto loop_rate = ros::Rate(10);

    auto trajectory_slope = stoi(argv[1]);

    goal.first = goal_x;
    goal.second = goal_y;

    auto odom_cb = [&](const nav_msgs::Odometry::ConstPtr& msg) {
        auto qx = msg->pose.pose.orientation.x;
        auto qy = msg->pose.pose.orientation.y;
        auto qz = msg->pose.pose.orientation.z;
        auto qw = msg->pose.pose.orientation.w;

        auto curr_x = msg->pose.pose.position.x;
        auto curr_y = msg->pose.pose.position.y;
        // Position error
        auto rho = sqrt(pow(curr_x - goal.first, 2) + pow(curr_y - goal.second, 2));

        // Desired heading
        auto theta = get<2>(quarternion_to_euler(qx, qy, qz, qw));      // vehicle yaw
        auto beta = -atan2(goal.second - curr_y, goal.first - curr_x);  // desired heading

        // Heading error
        auto alpha = -beta - theta;

        auto command = geometry_msgs::Twist();

        auto v = KP * rho;
        auto omega = KA * alpha; //+ KB * beta;

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