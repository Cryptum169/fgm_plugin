#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <vector>

void modelstatesCallback(const gazebo_msgs::ModelStates msg);
std::string robot_name = "mobile_base";
std::string child_id = "base_footprint";
geometry_msgs::PoseWithCovariance rbt_pose;
geometry_msgs::TwistWithCovariance rbt_twist;
ros::Publisher ground_truth;
nav_msgs::Odometry fake_odom;
std_msgs::Header fake_header;

int main(int argc, char ** argv) {
    ros::init(argc, argv, "count_and_log");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 10, &modelstatesCallback);
    ground_truth = nh.advertise<nav_msgs::Odometry>("ground_truth/odom", 10);
    fake_header.frame_id = "odom";
    fake_odom.child_frame_id = "base_footprint";
    fake_odom.header = fake_header;
    ROS_INFO_STREAM("Ground truth node initiated");
    ros::spin();
    return 0;
}

void modelstatesCallback(const gazebo_msgs::ModelStates msg) {
    for (std::vector<std::string>::size_type it = 0; it < msg.name.size(); ++it) {
        if (msg.name[it].compare(robot_name) == 0) {
            rbt_pose.pose = msg.pose[it];
            rbt_twist.twist = msg.twist[it];
            fake_odom.pose = rbt_pose;
            fake_odom.twist = rbt_twist;
            ground_truth.publish(fake_odom);
            break;
        }
    }
}