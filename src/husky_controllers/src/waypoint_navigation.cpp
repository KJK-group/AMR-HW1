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
#define KA 0.3
#define KB 0.01
#define POS_TOLERANCE 0.2
#define ANGLE_TOLERANCE 0.025

using namespace std;

auto goal = make_pair(0, 0);

pair<float, float> waypoints[] = {make_pair(11.5, 0.5), make_pair(11.5, 5.5), make_pair(4, 5.5), make_pair(0, 0)};

ros::Publisher pub;
ros::Subscriber sub;

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "first_order_trajectory_follower");

    auto nh = ros::NodeHandle("~");
    pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);

    auto loop_rate = ros::Rate(10);

    auto waypoint_index = 0;

    auto odom_cb = [&](const nav_msgs::Odometry::ConstPtr& msg) {
        if (waypoint_index > size(waypoints)) {
            sub.shutdown();
            return;
        }
        goal = waypoints[waypoint_index];

        auto curr_x = msg->pose.pose.position.x;
        auto curr_y = msg->pose.pose.position.y;

        // Position error
        auto rho = sqrt(pow(curr_x - goal.first, 2) + pow(curr_y - goal.second, 2));

        // Desired heading
        auto theta = tf::getYaw(msg->pose.pose.orientation);            // vehicle yaw
        auto beta = -atan2(goal.second - curr_y, goal.first - curr_x);  // desired heading

        // Heading error
        auto alpha = -beta - theta;

        auto command = geometry_msgs::Twist();

        auto v = KP * rho;
        auto omega = KA * alpha; //+ KB * beta;

        if (alpha > ANGLE_TOLERANCE) {
            command.linear.x = 0;
            command.angular.z = omega;
        }
        else if (rho > POS_TOLERANCE) {
            command.linear.x = v;
            command.angular.z = 0;
        }
        else {
            waypoint_index++;
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

    sub = nh.subscribe<nav_msgs::Odometry>("/husky_velocity_controller/odom", 10, odom_cb);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}