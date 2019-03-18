#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>
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

#ifndef PI
#define PI 3.1415926
#endif

PLUGINLIB_EXPORT_CLASS(fgm_plugin::FGMPlanner, nav_core::BaseLocalPlanner)

namespace fgm_plugin
{
    FGMPlanner::FGMPlanner() {
        ros::NodeHandle nh;
        ros::Publisher info_pub;
        ros::Publisher vis_pub;
        ros::Subscriber laser_sub;
        ros::Subscriber pose_sub;
        ros::Publisher gap_angle_pub;
        go_to_goal = false;
        gap_switch_counter = 0;
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
        // srv_(nh);
        alpha = 2.0;
        info_pub = nh.advertise<std_msgs::String>("planner_info", 100);
        // gap_angle_pub = nh.advertise<>
        vis_pub = nh.advertise<visualization_msgs::Marker>("/viz_array", 3000);
        laser_sub = nh.subscribe("/point_scan", 100, &FGMPlanner::laserScanCallback, this);
        pose_sub = nh.subscribe("/robot_pose",10, &FGMPlanner::poseCallback, this);

        f = boost::bind(&FGMPlanner::reconfigureCb, this, _1, _2);
        server.setCallback(f);

        ROS_INFO_STREAM("FGMPlanner initialized");

    }

    bool FGMPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
        // plan is the global plan to follow, assumes last entry of plan is the goal
        int plan_length = plan.size();
        int index = std::min(plan_length, 500);
        goal_pose = plan.at(index - 1);
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
        visualization_msgs::MarkerArray vis_arr;
        // used perfect localization

        double yaw = atan2(2.0 * (current_pose_.orientation.w * current_pose_.orientation.z + current_pose_.orientation.x * current_pose_.orientation.y),
            1.0 - 2.0 * (current_pose_.orientation.y * current_pose_.orientation.y + current_pose_.orientation.z * current_pose_.orientation.z));
        goal_angle = atan2(goal_pose.pose.position.y - current_pose_.position.y, goal_pose.pose.position.x - current_pose_.position.x);


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
        gap_angle = 0;

        // Compute gaps
        bool prev = true; // Take range of pov as wall
        float l_dist = 0;
        float r_dist = 0;
        int size = 0;
        int start_nan_idx = 144;
        dmin = 10;

        // Generating Gaps
        // Limited fov
        for(std::vector<float>::size_type it = 144; it < stored_scan_msgs.ranges.size() && it < 368; ++it)
        {
            if (prev) {
                if (stored_scan_msgs.ranges[it] > 2.9) { // isnan
                    ++size;
                } else {
                    r_dist = stored_scan_msgs.ranges[it];
                    dmin = fmin(dmin, r_dist);
                    if (size > 10) {
                        // Filter out noise, tho rarely exists
                        Gap newGap(start_nan_idx, it, size, l_dist, r_dist, goal_angle);
                        if (newGap.traversable() == 1) {
                            pq.push(newGap);
                        }
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

        // Account for error
        if (prev) {
            Gap placeHolder(start_nan_idx, 368 - 1, size, l_dist, 0, goal_angle);        
            pq.push(placeHolder);
        }

        // Calculate gap angle
        // Do Sticky gap implementation
        // NOT STICKY YET
        if (pq.size() != 0) {
            currLarge = pq.top();
            if (currLarge.getScore() < lastGap.getScore() / 1.5 && lastGap.getScore() > 0.1) {
                gap_angle = currLarge.getAngle();
                gap_switch_counter = 0;
                lastGap = currLarge;
            } else {
                if (gap_switch_counter < 3) {
                    gap_angle = lastGap.getAngle();
                    gap_switch_counter = 0;
                } else {
                    gap_angle = currLarge.getAngle();
                    gap_switch_counter = 0;
                    lastGap = currLarge;
                }
            }
            gap_angle = currLarge.getAngle();
        } else {
            ROS_DEBUG_STREAM("No Traversable gap found");
            // POTENTIAL HAZARD
        }

        // Questionable usefulness 
        {
            std::ostringstream ss;
            ss << gap_angle;
            std_msgs::String angle_data;
            angle_data.data = ss.str();
            info_pub.publish(angle_data);
        }

        pathVisualization(&vis_arr, gap_angle, 1);

        heading = (alpha / dmin * gap_angle + dmin * goal_angle)/(alpha / dmin + dmin);
        cmd_vel.linear.x = fmin(fabs(0.1 / heading), 0.6);
        heading = fmax(fmin(heading, 0.8), -0.8);
        cmd_vel.angular.z = 0.8 * heading;
        return true;
    }

    FGMPlanner::~FGMPlanner() {
        ROS_INFO_STREAM("Terminated");
    }

    void reconfigureCb(fgm_plugin::FGMConfig& config, uint32_t level) {
        ROS_INFO_STREAM(config.scan_height);
    }

    bool FGMPlanner::checkGoToGoal(float goal_angle) {
        // Or traversable gap
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
            if (stored_scan_msgs.ranges[access_idx] < std::min(2.9f, goalDistance())) {
                return false;
            };
        }

        float dist = 0;
        float min_clearance = 0;
        int max_check_idx = angleToSensorIdx(goal_angle - atan2(goalDistance(), 0.2));
        max_check_idx = max_check_idx < 0? max_check_idx += 512 : max_check_idx;
        int i = 0;
        // ROS_INFO_STREAM(max_check_idx);
        for (int j = goalIdx - 128; j < max_check_idx && j < goal_left_idx; j++) {
            access_idx = j < 0 ? j += 512 : j;
            dist = stored_scan_msgs.ranges[access_idx];
            min_clearance = dist * cos(i * stored_scan_msgs.angle_increment);
            if (min_clearance < 0.4) {
                return false;
            }
            i++;
        }

        i = 0;
        int min_check_idx = angleToSensorIdx(goal_angle + atan2(goalDistance(), 0.2));
        min_check_idx = min_check_idx > 512? min_check_idx -= 512 : min_check_idx;
        for (int j = goalIdx + 128; j > min_check_idx && j > goal_left_idx; j--) {
            access_idx = j < 0 ? j += 512 : j;
            access_idx = j > 512 ? j -= 512 : j;
            dist = stored_scan_msgs.ranges[access_idx];
            min_clearance = dist * cos(i * stored_scan_msgs.angle_increment);
            if (min_clearance < 0.4) {
                return false;
            }
            i++;
        }
        return true;
    }

    int FGMPlanner::angleToSensorIdx(float goal_angle) {
        return (int)((goal_angle - stored_scan_msgs.angle_min) / stored_scan_msgs.angle_increment);
    }

    float FGMPlanner::goalDistance(){
        return sqrt(pow(goal_pose.pose.position.y - current_pose_.position.y,2) + pow(goal_pose.pose.position.x - current_pose_.position.x,2));
    }

    void FGMPlanner::pathVisualization(visualization_msgs::MarkerArray* vis_arr, float degree, int hash) {
        for (int i = 0; i < 40; i++) {
            visualization_msgs::Marker marker;
            // marker. "/base_link"
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time();
            marker.id = hash * 100 + i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = i * 0.05 * cos(degree);
            marker.pose.position.y = i * 0.05 * sin(degree);
            marker.pose.position.z = 0.5;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            // marker.lifetime = ros::Duration(0.2);
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            if (hash = 0) {
                marker.color.r = 1.0;
            } else if (hash == 1) {
                marker.color.b = 1.0;
            } else {
                marker.color.g = 1.0;
            }
            vis_pub.publish(marker);
        }
        return;
    }

}