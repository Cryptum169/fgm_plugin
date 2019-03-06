#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>
#include <vector>
#include <queue>
#include <string>
#include <math.h>
#include <cmath>
// #include <math>
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
        ros::Subscriber pose_sub;
        ros::Publisher gap_angle_pub;
        go_to_goal = false;
        Gap lastGap();
    }

    void FGMPlanner::laserScanCallback(const sensor_msgs::LaserScan msg) {
        // Store local san message
        stored_scan_msgs = msg;
    }

    void FGMPlanner::poseCallback(const geometry_msgs::Pose msg) {
        // Robot Pose message
        current_pose_ = msg;
    }

    void FGMPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) {
        alpha = 2.0;
        info_pub = nh.advertise<std_msgs::String>("planner_info", 100);
        // gap_angle_pub = nh.advertise<>
        laser_sub = nh.subscribe("/point_scan", 100, &FGMPlanner::laserScanCallback, this);
        pose_sub = nh.subscribe("/robot_pose",10, &FGMPlanner::poseCallback, this);

        // costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        // planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());
        ROS_INFO_STREAM("FGMPlanner initialized");

    }

    bool FGMPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
        // plan is the global plan to follow, assumes last entry of plan is the goal
        goal_pose = plan.back();
        return true;
    }

    bool FGMPlanner::isGoalReached() {
        if (goalDistance() < 0.5) {
            ROS_INFO_STREAM("Goal Reached");
            Gap lastGap();
            return true;
        } else {
            return false;
        }
    }

    bool FGMPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
        // used perfect localization
        // sharedPtr_pose = ros::topic::waitForMessage<geometry_msgs::Pose>("/robot_pose", ros::Duration(1));
        // if (sharedPtr_pose == NULL) {
        //     ROS_ERROR("Planner received no odom!");
        // } else {
        //     current_pose_ = (*sharedPtr_pose);
        // }

        double yaw = atan2(2.0 * (current_pose_.orientation.w * current_pose_.orientation.z + current_pose_.orientation.x * current_pose_.orientation.y),
            1.0 - 2.0 * (current_pose_.orientation.y * current_pose_.orientation.y + current_pose_.orientation.z * current_pose_.orientation.z));
        goal_angle = atan2(goal_pose.pose.position.y - current_pose_.position.y, goal_pose.pose.position.x - current_pose_.position.x);

        // costmap_ros_->getRobotPose(current_pose_2);
        // double pose_x = current_pose_2.getOrigin().getX();
        // double pose_y = current_pose_2.getOrigin().getY();
        // double yaw = tf::getYaw(current_pose_2.getRotation());
        // goal_angle = atan2(goal_pose.pose.position.y - pose_y, goal_pose.pose.position.x - pose_y);
        yaw = fmod(yaw + 2 * PI, 2 * PI) - PI;
        goal_angle = fmod(goal_angle + 2 * PI, 2 * PI) - PI;
        goal_angle = goal_angle - yaw;
        if (goal_angle > 0) {
            if (goal_angle > PI) {
                goal_angle -= 2 * PI;
            }
        } else {
            if (goal_angle < -PI) {
                goal_angle += 2 * PI;
            }
        }
        // goal_angle = goal_angle > 0 ? fmod(goal_angle + 2 * PI, 2 * PI) : fmod(goal_angle - 2 * PI, 2 * PI);

        // ROS_INFO_STREAM("Goal Angle:");
        // ROS_INFO_STREAM(goal_angle);

        // Asus carried by turtlebot has 30cm diagnoal screen, let turtlebot clearance be 50cm

        bool goal_path_clear = checkGoToGoal(goal_angle);

        if (goal_path_clear) {
            heading = goal_angle;
            cmd_vel.linear.x = fmin(fabs(0.1 / heading), 0.6);
            // cmd_vel.linear.x = 0;
            heading = fmax(fmin(heading, 0.8), -0.8);
            cmd_vel.angular.z = 0.8 * heading;
            return true;
        }

        std::priority_queue <Gap, std::vector<Gap>, gapComparator> pq;        
        // This is a place holder
        Gap currLarge(0,0,0,0,0, goal_angle);
        // currLarge.setGoalAngle(goal_angle);
        // currLarge.setSensorModel(stored_scan_msgs.angle_increment, stored_scan_msgs.angle_min);
        gap_angle = 0;

        // Compute gaps
        bool prev = true; // Take range of pov as wall
        float l_dist = 0;
        float r_dist = 0;
        int size = 0;
        int start_nan_idx = 144;
        dmin = 10;

        // Generating Gaps
        for(std::vector<float>::size_type it = 144; it < stored_scan_msgs.ranges.size() && it < 368; ++it)
        {
            if (prev) {
                if (stored_scan_msgs.ranges[it] > 2.9) { // isnan
                    ++size;
                } else {
                    r_dist = stored_scan_msgs.ranges[it];
                    dmin = fmin(dmin, r_dist);
                    if (size > 2) {
                        // Filter out noise, tho rarely exists
                        // Gap newGap(start_nan_idx, it, size, l_dist, r_dist, goal_angle);
                        // if (newGap.traversable() == 1) {
                        // pq.push(newGap);
                        // }
                        pq.push(Gap(start_nan_idx, it, size, l_dist, r_dist, goal_angle));
                    }
                    start_nan_idx = -1;
                    size = 0;
                    prev = false;
                }
            } else {
                if (stored_scan_msgs.ranges[it] > 2.9) { // isnan
                    start_nan_idx = it;
                    size = 1;
                    prev = true;
                } else {
                    l_dist = stored_scan_msgs.ranges[it];
                    dmin = fmin(dmin, l_dist);
                }
            }
            // Populate obstacle here
        }

        if (prev) {
            Gap placeHolder(start_nan_idx, 368 - 1, size, l_dist, 0, goal_angle);        
            // pq.push(Gap(start_nan_idx, stored_scan_msgs.ranges.size() -1, size, l_dist, 0, goal_angle));
            pq.push(placeHolder);
            // ROS_INFO_STREAM("No gap detected");
            // ROS_INFO_STREAM(placeHolder.getAngle());
        }

        ROS_INFO_STREAM(pq.size());
        
        // Calculate gap angle
        if (pq.size() != 0) {
            currLarge = pq.top();
            if (currLarge.getScore() < lastGap.getScore() / 1.5 && lastGap.getScore() > 0.1) {
                gap_angle = currLarge.getAngle();
                lastGap = currLarge;
                // ROS_INFO_STREAM("switched to");
                // ROS_INFO_STREAM(currLarge.getAngle());
            } else {
                // ROS_INFO_STREAM("not switched");
                // ROS_INFO_STREAM(lastGap.getAngle());
                gap_angle = lastGap.getAngle();
            }
        } else {
            ROS_INFO_STREAM("No Traversable gap found");
        }

        ROS_INFO_STREAM("-------");
        while (! pq.empty()) {
            Gap this_gap = pq.top();
            pq.pop();
            ROS_INFO_STREAM(this_gap.getAngle());
        }
        ROS_INFO_STREAM("-------");

        ROS_INFO_STREAM(gap_angle);
        ROS_INFO_STREAM("-------");
        ROS_INFO_STREAM(goal_angle);
        ROS_INFO_STREAM("_______");

        {
            std::ostringstream ss;
            ss << gap_angle;
            std_msgs::String angle_data;
            angle_data.data = ss.str();
            info_pub.publish(angle_data);
        }


        heading = (alpha / dmin * gap_angle + dmin * goal_angle)/(alpha / dmin + dmin);
        // ROS_INFO_STREAM(heading);
        // heading = goal_angle;
        cmd_vel.linear.x = fmin(fabs(0.1 / heading), 0.6);
        // cmd_vel.linear.x = 0.0;
        heading = fmax(fmin(heading, 0.8), -0.8);
        // heading = 0;
        cmd_vel.angular.z = 0.8 * heading;
   
        return true;
    }

    FGMPlanner::~FGMPlanner() {
        ROS_INFO_STREAM("Terminated");
    }

    bool FGMPlanner::checkGoToGoal(float goal_angle) {
        // Or traversable gap

        bool return_value = true;
        int goalIdx = angleToSensorIdx(goal_angle);
        int goal_left_idx = angleToSensorIdx(goal_angle - asin(0.25/3));
        int goal_right_idx = angleToSensorIdx(goal_angle + asin(0.25/3));
        int access_idx = 0;

        for (int j = goal_left_idx; j < goal_right_idx; ++j) {
            if (j < 0) {
                access_idx = j + 512;
            } else {
                access_idx = j;
            }

            return_value = return_value && (stored_scan_msgs.ranges[access_idx] > std::min(2.9f, goalDistance()));
        }

        return return_value;
        
        // TODO:
        // Need to rework to check for entire path,
        // go from goalIdx - 128 to goalIdx + 128, wrap around if necessary
    }

    int FGMPlanner::angleToSensorIdx(float goal_angle) {
        return (int)((goal_angle - stored_scan_msgs.angle_min) / stored_scan_msgs.angle_increment);
    }

    float FGMPlanner::goalDistance(){
        return sqrt(pow(goal_pose.pose.position.y - current_pose_.position.y,2) + pow(goal_pose.pose.position.x - current_pose_.position.x,2));
    }

}