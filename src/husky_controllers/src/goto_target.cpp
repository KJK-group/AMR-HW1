#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <cmath>
#include <tuple>
#include <iostream>
#include <utility>
#include <string>
#include <algorithm>
#include "husky_controllers/Error.h"

#define POS_TOLERANCE 0.2

using namespace std;

typedef tuple<float, float, float> triple;

// target point
auto goal = make_pair(0, 0);
// controller gains
auto K_rho = 0.2f;
auto K_alpha = 0.5f;
auto K_beta = 0.f;

ros::Publisher pub_control;
ros::Publisher pub_error;
ros::Subscriber sub_odom;

auto odometry_filtered_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void {
    // vehicle 2D position
    auto curr_x = msg->pose.pose.position.x;
    auto curr_y = msg->pose.pose.position.y;

    // euclidean position error
    auto rho = sqrt(pow(curr_x - goal.first, 2) + pow(curr_y - goal.second, 2));

    // position error publishing
    auto error_msg = husky_controllers::Error();
    error_msg.error = rho;
    pub_error.publish(error_msg);

    // heading
    auto theta = tf::getYaw(msg->pose.pose.orientation);           // vehicle yaw
    auto beta = -atan2(goal.second - curr_y, goal.first - curr_x); // desired heading

    // heading error
    auto alpha = -beta - theta;
    // correct yaw in range [-pi, pi]
    if (alpha > M_PI) {
        alpha -= 2*M_PI;
    }
    else if (alpha < -M_PI) {
        alpha += 2*M_PI;
    }

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
    // ROS initialisations
    ros::init(argc, argv, "husky_goto_target");
    auto nh = ros::NodeHandle("~");

    // control publisher to cmd_vel
    pub_control = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);
    // error (rho) publisher to custom topic
    pub_error = nh.advertise<husky_controllers::Error>("/husky_controllers/position_error", 10);

    // passing command line arguments
    // goal coordinate pair
    if (argc > 1) goal.first = stoi(argv[1]);
    if (argc > 2) goal.second = stoi(argv[2]);
    // controller gains
    if (argc > 3) K_rho = stof(argv[3]);
    if (argc > 4) K_alpha = stof(argv[4]);
    if (argc > 5) K_beta = stof(argv[5]);

    // subscriber to odometry filtered data from the husky
    sub_odom = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, odometry_filtered_cb);

    auto loop_rate = ros::Rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}