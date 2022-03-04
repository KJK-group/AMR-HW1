#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <memory>

auto camera2_pose_pub = std::unique_ptr<ros::Publisher>();
auto odometry_filtered_sub = std::unique_ptr<ros::Subscriber>();
auto tfbuf = std::shared_ptr<tf2_ros::Buffer>();

auto odometry_filtered_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void {
    try {
        auto transform = tfbuf->lookupTransform("camera_2_link", msg->header.frame_id, ros::Time(0));
        auto from = geometry_msgs::PoseWithCovarianceStamped();
    from.header = msg->header;
    from.pose = msg->pose;

    auto to = geometry_msgs::PoseWithCovarianceStamped();
    tf2::doTransform(to, from, transform);
    to.header.frame_id = "camera_2_link";

    camera2_pose_pub->publish(to);

    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
}

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "camera2_pose_publisher");
    auto nh = ros::NodeHandle("~");

    tfbuf = std::make_shared<tf2_ros::Buffer>();
    auto tf_listener = tf2_ros::TransformListener(*tfbuf);

    camera2_pose_pub = std::make_unique<ros::Publisher>(nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/husky_controllers/camera2/pose", 10));
    odometry_filtered_sub = std::make_unique<ros::Subscriber>(nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, odometry_filtered_cb));
    auto loop_rate = ros::Rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}