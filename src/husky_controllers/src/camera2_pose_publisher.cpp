#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/tf2.h>

ros::Publisher camera2_pose_pub;
ros::Subscriber odometry_filtered_sub;

auto odometry_filtered_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void {

}

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "camera2_pose_publisher");

    auto nh = ros::NodeHandle("~");
    odometry_filtered_previous_time = ros::Time::now();

    camera2_pose_pub = nh.advertise<nav_msgs::Odometry>("/husky_controllers/camera2/pose", 10);

    auto loop_rate = ros::Rate(10);

    odometry_filtered_sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, odometry_filtered_cb);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}