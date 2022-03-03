#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>


ros::Publisher camera2_pose_pub;
ros::Subscriber odometry_filtered_sub;

tf2_ros::Buffer* tfBuffer;
//tf2_ros::TransformListener *tfListener;

auto odometry_filtered_cb(const nav_msgs::Odometry::ConstPtr& msg) -> void {
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer->lookupTransform("camera_2_link", "base_link", ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    auto camera_2_odometry = nav_msgs::Odometry();
    // Transform
    camera_2_odometry.pose.pose.position.x = msg->pose.pose.position.x + transformStamped.transform.translation.x;
    camera_2_odometry.pose.pose.position.y = msg->pose.pose.position.y + transformStamped.transform.translation.y;
    camera_2_odometry.pose.pose.position.z = msg->pose.pose.position.z + transformStamped.transform.translation.z;

    // Orientation / rotation
    auto q_origin = tf2::Quaternion(
        msg->pose.pose.orientation.x, 
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    //tf2::fromMsg(msg->pose.pose.orientation, q_origin);

    auto q_rot = tf2::Quaternion(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w
    );
    //tf2::fromMsg(transformStamped.transform.rotation, q_rot);

    auto q_new = q_rot*q_origin;  // Calculate the new orientation / http://wiki.ros.org/tf2/Tutorials/Quaternions
    q_new.normalize();

    //tf2::convert(q_new, camera_2_odometry.pose.pose.orientation);
    camera_2_odometry.pose.pose.orientation.x = q_new.x();
    camera_2_odometry.pose.pose.orientation.y = q_new.y();
    camera_2_odometry.pose.pose.orientation.z = q_new.z();
    camera_2_odometry.pose.pose.orientation.w = q_new.w();

    camera_2_odometry.twist  = msg->twist;
    camera_2_odometry.child_frame_id = "camera_2_link";

    auto header = msg->header;
    header.frame_id = "base_link";
        camera_2_odometry.header = msg->header;

    camera_2_odometry.pose.covariance = msg->pose.covariance;

    camera2_pose_pub.publish(camera_2_odometry);
}

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "camera2_pose_publisher");

    auto nh = ros::NodeHandle("~");
    tf2_ros::Buffer _tfBuffer;
    tf2_ros::TransformListener _tfListener(_tfBuffer);
    tfBuffer = &_tfBuffer;

    camera2_pose_pub = nh.advertise<nav_msgs::Odometry>("/husky_controllers/camera2/pose", 10);

    auto loop_rate = ros::Rate(10);

    odometry_filtered_sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 10, odometry_filtered_cb);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}