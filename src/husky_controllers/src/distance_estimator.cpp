#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <cmath>
#include "husky_controllers/AccumulatedDistance.h"

using husky_controllers::AccumulatedDistance;
using std::pow;
using std::abs;

ros::Publisher odometry_filtered_pub;
ros::Publisher model_states_pub;
ros::Subscriber odometry_filtered_sub;
ros::Subscriber model_states_sub;

auto odometry_filtered_distance = 0.f;
ros::Time odometry_filtered_previous_time;

auto model_states_distance = 0.f;
ros::Time model_states_previous_time;

auto odometry_filtered_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void {
    auto now = ros::Time::now();
    auto delta_time = now - odometry_filtered_previous_time;
    auto delta_distance = msg->twist.twist.linear.x * delta_time.toNSec()/pow(10, 9);
    odometry_filtered_distance += delta_distance;
    odometry_filtered_previous_time = now;

    auto distance_msg = AccumulatedDistance();
    distance_msg.distance = odometry_filtered_distance;
    odometry_filtered_pub.publish(distance_msg);
}

auto model_states_cb(const gazebo_msgs::ModelStates::ConstPtr& msg) -> void {
    auto now = ros::Time::now();
    auto delta_time = now - model_states_previous_time;
    ROS_INFO("size: %ld", std::size(msg->twist));
    // Model states return 3 states from the world and the state of the husky robot is at index 2
    // MOdel states is measured in world frame, as such velocity can be negative
    auto delta_distance = (abs(msg->twist[2].linear.x) + abs(msg->twist[2].linear.y)) * delta_time.toNSec()/pow(10, 9);
    model_states_distance += delta_distance;
    model_states_previous_time = now;

    auto distance_msg = AccumulatedDistance();
    distance_msg.distance = model_states_distance;
    model_states_pub.publish(distance_msg);
}

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "distance_estimator");

    auto nh = ros::NodeHandle("~");
    odometry_filtered_previous_time = ros::Time::now();
    model_states_previous_time = ros::Time::now();

    odometry_filtered_pub = nh.advertise<husky_controllers::AccumulatedDistance>("/husky_controllers/odometer/odometry_filtered", 10);
    model_states_pub = nh.advertise<husky_controllers::AccumulatedDistance>("/husky_controllers/odometer/model_states", 10);

    auto loop_rate = ros::Rate(10);

    odometry_filtered_sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, odometry_filtered_cb);
    model_states_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, model_states_cb);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}