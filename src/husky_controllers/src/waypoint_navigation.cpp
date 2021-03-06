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

#define KP 1
#define KA 1
#define KB 0
#define POS_TOLERANCE 0.1
#define ANGLE_TOLERANCE 0.02

using namespace std;

auto goal = make_pair(0, 0); // Goal consists of x and y coordinates
// The following coordinates are defined to get the husky thoug the maze
pair<float, float> waypoints[] = {make_pair(11.5, 0.5), make_pair(11.5, 4.5), make_pair(4.5, 4.5), make_pair(0, 0)};

auto driving = false;

ros::Publisher pub;
ros::Subscriber sub;

auto main(int argc, char **argv) -> int
{
    // Initialise the node and its manager
    ros::init(argc, argv, "waypoint_navigation");
    auto nh = ros::NodeHandle("~");
    pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);

    auto loop_rate = ros::Rate(10);

    auto waypoint_index = 0;

    auto odom_cb = [&](const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (waypoint_index > size(waypoints))
        {
            sub.shutdown();
            return;
        }
        goal = waypoints[waypoint_index];

        auto curr_x = msg->pose.pose.position.x;
        auto curr_y = msg->pose.pose.position.y;

        // euclidean position error
        auto rho = sqrt(pow(curr_x - goal.first, 2) + pow(curr_y - goal.second, 2));

        // Desired heading
        auto theta = tf::getYaw(msg->pose.pose.orientation);          // Vehicle yaw
        auto beta = atan2(goal.second - curr_y, goal.first - curr_x); // Desired heading

        auto alpha = beta - theta;
        if (alpha > M_PI)
        {
            alpha -= 2 * M_PI;
        }
        else if (alpha < -M_PI)
        {
            alpha += 2 * M_PI;
        }
        // Heading error
        // if (theta < 0 and beta >= 0) {
        //     alpha = 2 * M_PI - alpha;
        // }
        // else if (theta >= 0 and beta < 0) {
        //     alpha = 2 * M_PI + alpha;
        // }
        // alpha = alpha > M_PI ? M_PI - alpha : alpha;

        auto command = geometry_msgs::Twist();

        auto v = KP * rho;
        auto omega = KA * alpha + KB * beta;

        ROS_WARN("state: ");
        if (abs(alpha) > ANGLE_TOLERANCE && !driving)
        {
            ROS_WARN("  turning");
            command.linear.x = 0;
            command.angular.z = omega;
        }
        else if (rho > POS_TOLERANCE)
        {
            ROS_WARN("  driving");
            driving = true;
            command.linear.x = v;
            command.angular.z = omega;
        }
        else
        {
            ROS_WARN("  stopping");
            driving = false;
            waypoint_index++;
            command.linear.x = 0;
            command.angular.z = 0;
        }

        // ROS_INFO("pose:");
        // ROS_INFO("  x:     %.5f", curr_x);
        // ROS_INFO("  y:     %.5f", curr_y);
        // ROS_INFO("  theta: %.5f", theta);

        // ROS_INFO("errors:");
        // ROS_INFO("  rho:   %.5f", rho);
        // ROS_INFO("  beta:  %.5f", beta);
        // ROS_INFO("  alpha: %.5f", alpha);

        // ROS_INFO("control outputs:");
        // ROS_INFO("  v:     %.5f", v);
        // ROS_INFO("  omega: %.5f", omega);

        pub.publish(command);
    };

    sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, odom_cb);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}