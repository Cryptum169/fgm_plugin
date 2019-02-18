#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>
#include <vector>
#include <queue>
#include <string>
#include <math.h>
#include <cmath>
#include <sstream>
#include "std_msgs/String.h"
#include <stdlib.h>
#include <fgm_plugin/gap.h>
#include <fgm_plugin/fgm_planner.h>
#include <fgm_plugin/gap_comparator.h>

#define PI 3.1415926

PLUGINLIB_EXPORT_CLASS(fgm_plugin::FGMPlanner, nav_core::BaseLocalPlanner)

namespace fgm_plugin
{
    FGMPlanner::FGMPlanner() {
        ros::NodeHandle nh;
        ros::Publisher info_pub;
        ros::Subscriber laser_sub;
    }


    void FGMPlanner::laserScanCallback(const sensor_msgs::LaserScan msg) {
        // Store local san message
        stored_scan_msgs = msg;
    }

    void FGMPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) {
        alpha = 2.0;
        info_pub = nh.advertise<std_msgs::String>("planner_info", 100);
        laser_sub = nh.subscribe("/scan", 100, &FGMPlanner::laserScanCallback, this);

        costmap_ros_ = costmap_ros;
        costmap_ros_->getRobotPose(current_pose_2);
        char buffer [50];
        int n=sprintf(buffer, "Robot Start Position: %f, %f, %f\n",current_pose_2.getOrigin().getX(), current_pose_2.getOrigin().getY(), tf::getYaw(current_pose_2.getRotation()));
        ROS_INFO_STREAM(buffer);

        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());
        ROS_INFO_STREAM("FGMPlanner initialized");
        // ros::spin(); // Only spin at the right level!
        // Spin at the right place
    }

    bool FGMPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
        // ROS_INFO_STREAM(plan.back());
        // plan is the global plan to follow, assumes last entry of plan is the goal
        goal_pose = plan.back();
        return true;
    }

    bool FGMPlanner::isGoalReached() {
        if (sqrt(pow(goal_pose.pose.position.y - current_pose_.position.y,2) + pow(goal_pose.pose.position.x - current_pose_.position.x,2)) < 0.5) {
            ROS_INFO_STREAM("Goal Reached");
            return true;
        } else {
            return false;
        }
    }

    bool FGMPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {

        // used perfect localization, odom retrieved from gazebo/model_states
        // sharedPtr_pose = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", ros::Duration(1));
        sharedPtr_pose = ros::topic::waitForMessage<geometry_msgs::Pose>("/robot_pose", ros::Duration(1));
        if (sharedPtr_pose == NULL) {
            ROS_ERROR("Planner received no odom!");
        } else {
            current_pose_ = *sharedPtr_pose;
        }
        
        // ROS_INFO_STREAM("comvel called");
        // costmap_ros_->getRobotPose(current_pose_);
        // ROS_INFO_STREAM("Robot initial pose:" << current_pose_.getOrigin().getX() << ", " << current_pose_.getOrigin().getY() << ", " << tf::getYaw(current_pose_.getRotation()));

        double yaw = atan2(2.0 * (current_pose_.orientation.w * current_pose_.orientation.z + current_pose_.orientation.x * current_pose_.orientation.y),
            1.0 - 2.0 * (current_pose_.orientation.y * current_pose_.orientation.y + current_pose_.orientation.z * current_pose_.orientation.z));
        goal_angle = atan2(goal_pose.pose.position.y - current_pose_.position.y, goal_pose.pose.position.x - current_pose_.position.x);
        yaw = fmod(yaw + 2 * PI, 2 * PI);
        goal_angle = fmod(goal_angle + 2 * PI, 2 * PI);
        goal_angle = goal_angle - yaw;

        std::priority_queue <Gap, std::vector<Gap>, gapComparator> pq;        
        Gap currLarge(0,0,0,0,0);
        gap_angle = 0;

        // Compute gaps
        bool prev = true; // Take range of pov as wall
        float l_dist = 0;
        float r_dist = 0;
        int size = 0;
        int start_nan_idx = 0;
        dmin = 10;

        for(std::vector<float>::size_type it = 70; it < 200; ++it)
        {
            if (prev) {
                if (stored_scan_msgs.ranges[it] != stored_scan_msgs.ranges[it] || stored_scan_msgs.ranges[it] > 2.0) {
                    ++size;
                } else {
                    dmin = fmin(dmin, stored_scan_msgs.ranges[it]);
                    if (size > 20) {
                        // Be reworked to populate the obstacle size
                        pq.push(Gap(start_nan_idx, size, 0));
                    }
                    start_nan_idx = -1;
                    size = 0;
                    prev = false;
                }
            } else {
                if (stored_scan_msgs.ranges[it] != stored_scan_msgs.ranges[it] || stored_scan_msgs.ranges[it] > 2.0) {
                    start_nan_idx = it;
                    size = 1;
                    prev = true;
                } else {
                    dmin = fmin(dmin, stored_scan_msgs.ranges[it]);
                }
            }
            // Populate obstacle here
        }
        
        if (prev) {
            pq.push(Gap(start_nan_idx, stored_scan_msgs.ranges.size() -1, size, l_dist, 0));
        }
        
        // Calculate gap angle
        if (pq.size() != 0) {
            currLarge = pq.top();
            currLarge.setSensorModel(stored_scan_msgs.angle_increment, stored_scan_msgs.angle_min);
            gap_angle = currLarge.getAngle();
        }

        heading = (alpha / dmin * gap_angle + dmin * goal_angle)/(alpha / dmin + 1);
        cmd_vel.linear.x = fmin(fabs(0.2 / heading), 0.8);
        heading = fmax(fmin(heading, 0.8), -0.8);
        cmd_vel.angular.z = 0.8 * heading;
        return true;
    }

    FGMPlanner::~FGMPlanner() {
        ROS_INFO_STREAM("Terminated");
    }

}