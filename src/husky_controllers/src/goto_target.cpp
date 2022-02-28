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

#define KP 0.2
#define KA 0.5
#define KB 0.01
#define POS_TOLERANCE 0.2

using namespace std;

typedef tuple<float, float, float> triple;

auto goal = make_pair(0, 0);

auto rad_to_deg(float rad) -> float {
    return rad * 180/M_PI;
}

auto quarternion_to_euler(float x, float y, float z, float w) -> triple {
    // float t0 = 2.0 * (w * x + y * z);
    // float t1 = 1.0 - 2.0 * (x * x + y * y);
    // float X = rad_to_deg(atan2(t0, t1));

    // float t2 = 2.0 * (w * y - z * x);
    // t2 = t2 > 1.0 ? 1.0 : t2;
    // t2 = t2 < -1.0 ? -1.0 : t2;
    // float Y = rad_to_deg(asin(t2));

    // float t3 = 2.0 * (w * z + x * y);
    // float t4 = 1.0 - 2.0 * (y * y + z * z);
    // float Z = rad_to_deg(atan2(t3, t4));

    // X = atan2(t0, t1);
    // Y = asin(t2);
    // Z = atan2(t3, t4);

    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    double roll, pitch, theta;
    m.getRPY(roll, pitch, theta);

    return make_tuple(roll, pitch, theta);
}

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "husky_goto_target");

    auto euler = quarternion_to_euler(4.1, 6.2, 7.4, 2.9);
    ROS_WARN("x: %.5f\ty: %.5f\tz: %.5f", get<0>(euler), get<1>(euler), get<2>(euler));

    auto nh = ros::NodeHandle("~");
    auto pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);

    auto loop_rate = ros::Rate(10);

    auto goal_x = stoi(argv[1]);
    auto goal_y = stoi(argv[2]);

    goal.first = goal_x;
    goal.second = goal_y;

    auto odom_cb = [&](const nav_msgs::Odometry::ConstPtr& msg) {
    //auto odom_cb = [&](const gazebo_msgs::ModelStates::ConstPtr& msg) {
        auto qx = msg->pose.pose.orientation.x;
        auto qy = msg->pose.pose.orientation.y;
        auto qz = msg->pose.pose.orientation.z;
        auto qw = msg->pose.pose.orientation.w;
        // auto qx = msg->pose[sizeof(msg->pose)/sizeof(msg.pose[0])].orientation.x;
        // auto qy = msg->pose[sizeof(msg->pose)/sizeof(msg.pose[0])].orientation.y;
        // auto qz = msg->pose[sizeof(msg->pose)/sizeof(msg.pose[0])].orientation.z;
        // auto qw = msg->pose[sizeof(msg->pose)/sizeof(msg.pose[0])].orientation.w;

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
        // ROS_INFO_STREAM(setprecision(5) << fixed);
        
        // ROS_INFO_STREAM("pose:");
        // ROS_INFO_STREAM(setw(9) << left << "  x:" << setprecision(5) << curr_x);
        // ROS_INFO_STREAM(setw(9) << left << "  y:" << setprecision(5) << curr_y);
        // ROS_INFO_STREAM(setw(9) << left << "  theta:" << setprecision(5) << theta);

        // ROS_INFO_STREAM("errors:");
        // ROS_INFO_STREAM(setw(9) << left << "  rho:" << setprecision(5) << rho);
        // ROS_INFO_STREAM(setw(9) << left << "  beta:" << setprecision(5) << beta);
        // ROS_INFO_STREAM(setw(9) << left << "  alpha:" << setprecision(5) << alpha);

        pub.publish(command);
    };

    auto sub = nh.subscribe<nav_msgs::Odometry>("/husky_velocity_controller/odom", 10, odom_cb);
    //auto sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, odom_cb);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}