#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
//#include <LinearMath/btMatrix3x3.h>
#include <cmath>
#include <tuple>
#include <iostream>
#include <utility>
#include <string>
#include <algorithm>
//#include <Eigen/Geometry>
#include "husky_controllers/Error.h"

#define KP 0.2
#define KA 0.5
#define KB 0
#define POS_TOLERANCE 0.2

using namespace std;

typedef tuple<float, float, float> triple;

auto goal = make_pair(0, 0);

auto rad_to_deg(float rad) -> float {
    return rad * 180/M_PI;
}

auto main(int argc, char** argv) -> int {
    // ROS initialisations
    ros::init(argc, argv, "husky_goto_target");

    auto nh = ros::NodeHandle("~");
    // publisher to cmd_vel for the husky
    auto pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);
    auto pub_error = nh.advertise<husky_controllers::Error>("/husky_controllers/position_error", 10);
    auto loop_rate = ros::Rate(10);

    // passing command line arguments into goal coordinate pair
    goal.first = stoi(argv[1]);
    goal.second = stoi(argv[2]);

    auto odom_cb = [&](const nav_msgs::Odometry::ConstPtr& msg) {
        // Vehicle 2D position
        auto curr_x = msg->pose.pose.position.x;
        auto curr_y = msg->pose.pose.position.y;

        // Position error
        auto rho = sqrt(pow(curr_x - goal.first, 2) + pow(curr_y - goal.second, 2));    // Euclidian position error
        auto error_msg = husky_controllers::Error(); // Instantiate custom Error message
        error_msg.error = rho;
        pub_error.publish(error_msg);

        // Heading
        auto theta = tf::getYaw(msg->pose.pose.orientation);            // vehicle yaw
        auto beta = -atan2(goal.second - curr_y, goal.first - curr_x);  // desired heading

        // Heading error
        auto alpha = -beta - theta;

        auto command = geometry_msgs::Twist();

        auto v = KP * rho;
        auto omega = KA * alpha + KB * beta;

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

        ROS_INFO("control outputs:");
        ROS_INFO("  v:     %.5f", v);
        ROS_INFO("  omega: %.5f", omega);

        pub.publish(command);
    };

    auto sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, odom_cb);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}