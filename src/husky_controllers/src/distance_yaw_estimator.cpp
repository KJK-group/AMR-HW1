#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <cmath>
#include "husky_controllers/AccumulatedDistance.h"
#include "husky_controllers/Yaw.h"
#include <tf/transform_datatypes.h>

using husky_controllers::AccumulatedDistance;
using husky_controllers::Yaw;
using std::abs;
using std::pow;

ros::Publisher odometry_filtered_distance_pub;
ros::Publisher model_states_distance_pub;
ros::Publisher odometry_filtered_yaw_pub;
ros::Publisher model_states_yaw_pub;
ros::Subscriber odometry_filtered_sub;
ros::Subscriber model_states_sub;

auto odometry_filtered_distance = 0.f;
ros::Time odometry_filtered_previous_time;

auto model_states_distance = 0.f;
ros::Time model_states_previous_time;

// auto odometry_filtered_yaw = 0.f;
// auto model_states_yaw = 0.f;

auto odometry_filtered_cb(const nav_msgs::Odometry::ConstPtr &msg) -> void
{
    // Distance
    auto now = ros::Time::now(); // Used to calculate the time delta and is later defined as the previous time
    auto delta_time = now - odometry_filtered_previous_time;
    auto delta_distance = msg->twist.twist.linear.x * delta_time.toNSec() / pow(10, 9);
    odometry_filtered_distance += delta_distance;
    odometry_filtered_previous_time = now;

    // Publish the estimated distance travelled
    auto distance_msg = AccumulatedDistance();
    distance_msg.distance = odometry_filtered_distance;
    odometry_filtered_distance_pub.publish(distance_msg);

    // Estimated yaw
    auto yaw_msg = Yaw();
    yaw_msg.yaw = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("OF Yaw: %.5f", yaw_msg.yaw);

    // Publish the yaw
    odometry_filtered_yaw_pub.publish(yaw_msg);
}

auto model_states_cb(const gazebo_msgs::ModelStates::ConstPtr &msg) -> void
{
    auto now = ros::Time::now();
    auto delta_time = now - model_states_previous_time;
    //  Model states return 3 states from the world and the state of the husky robot is at index 2
    //  MOdel states is measured in world frame, as such velocity can be negative
    auto delta_distance = (abs(msg->twist[2].linear.x) + abs(msg->twist[2].linear.y)) * delta_time.toNSec() / pow(10, 9);
    model_states_distance += delta_distance;
    model_states_previous_time = now;

    auto distance_msg = AccumulatedDistance();
    distance_msg.distance = model_states_distance;
    // Publishing the ground truth distance travelled
    model_states_distance_pub.publish(distance_msg);

    // True yaw
    auto yaw_msg = Yaw();
    yaw_msg.yaw = tf::getYaw(msg->pose[2].orientation);
    ROS_INFO("MS Yaw: %.5f", yaw_msg.yaw);

    // Publishing the yaw
    model_states_yaw_pub.publish(yaw_msg);
}

auto main(int argc, char **argv) -> int
{
    ros::init(argc, argv, "distance_yaw_estimator"); // Initialise the node

    auto nh = ros::NodeHandle("~"); // Node manager

    // Initialising time parameters
    odometry_filtered_previous_time = ros::Time::now();
    model_states_previous_time = ros::Time::now();

    // Initialising publishers
    odometry_filtered_distance_pub = nh.advertise<AccumulatedDistance>("/husky_controllers/odometer/odometry_filtered", 10);
    model_states_distance_pub = nh.advertise<AccumulatedDistance>("/husky_controllers/odometer/model_states", 10);
    odometry_filtered_yaw_pub = nh.advertise<Yaw>("/husky_controllers/yaw/odometry_filtered", 10);
    model_states_yaw_pub = nh.advertise<Yaw>("/husky_controllers/yaw/model_states", 10);

    auto loop_rate = ros::Rate(10); // Publishing rate set to 10 Hz

    // Initialising the subscribers
    odometry_filtered_sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, odometry_filtered_cb);
    model_states_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, model_states_cb);

    while (ros::ok()) // Checking if node is avaliable
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}