#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_list_macros.h>
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

#ifndef PI
#define PI 3.1415926
#endif

PLUGINLIB_EXPORT_CLASS(fgm_plugin::FGMPlanner, nav_core::BaseLocalPlanner)

namespace fgm_plugin
{
    FGMPlanner::FGMPlanner() {
        ros::NodeHandle nh("~/fgm_planner_node");
        ros::Publisher info_pub;
        ros::Publisher vis_pub;
        ros::Subscriber laser_sub;
        ros::Subscriber pose_sub;
        ros::Subscriber pose_pub;
        ros::Publisher gap_angle_pub;
        go_to_goal = false;
        gap_switch_counter = 0;
        dynamic_recfg_server = boost::make_shared<dynamic_reconfigure::Server<fgm_plugin::FGMConfig> >(nh);
        Gap lastGap();
        path_count = 0;
    }

    void FGMPlanner::laserScanCallback(const sensor_msgs::LaserScan msg) {
        // Store local san message
        stored_scan_msgs = msg;
    }

    void FGMPlanner::poseCallback(const geometry_msgs::Pose msg) {
        // Robot Pose message
        current_pose_ = msg;
    }

    void FGMPlanner::reconfigureCb(fgm_plugin::FGMConfig& config, uint32_t level) {
        // Reconfigure Parameter
        max_linear_x = (float)config.max_linear;
        max_angular_z = (float)config.max_angular;
        fov = (float)config.fov;
        go_to_goal = config.go_to_goal;
        sub_goal_idx = config.sub_goal_idx;
        goal_distance_tolerance = (float)config.goal_tolerance;
        alpha = (float)config.alpha;
        score = config.score;

        ROS_DEBUG("Reconfigure:\n"
        "Max Linear: %f\n"
        "Max Angular: %f\n"
        "Field of View: %f\n"
        "Go to Goal Behavior: %s\n"
        "Scoring Function Behavior: %s\n"
        "Subgoal Index: %d\n"
        "Goal Test Tolerance: %f\n"
        "Alpha: %f",max_linear_x,max_angular_z,fov,go_to_goal? "True":"False",
        score? "True":"False", sub_goal_idx, goal_distance_tolerance, alpha);
        start_index = 256 - fov / 360 * 512 / 2;
    }

    void FGMPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) {
        info_pub = nh.advertise<std_msgs::String>("planner_info", 100);
        vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/viz_array", 3000);
        laser_sub = nh.subscribe("/point_scan", 100, &FGMPlanner::laserScanCallback, this);
        pose_sub = nh.subscribe("/robot_pose",10, &FGMPlanner::poseCallback, this);
        pose_pub = nh.advertise<geometry_msgs::PoseArray>("/path_array", 3000);


        f = boost::bind(&FGMPlanner::reconfigureCb, this, _1, _2);
        dynamic_recfg_server->setCallback(f);
        
        ROS_DEBUG("FGMPlanner initialized");
    }

    bool FGMPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
        int plan_length = plan.size();
        int index = std::min(plan_length, sub_goal_idx);
        goal_pose = plan.at(index - 1);
        return true;
    }

    bool FGMPlanner::isGoalReached() {
        if (goalDistance() < goal_distance_tolerance) {
            ROS_DEBUG("Goal Reached");
            Gap lastGap();
            traversed_path = geometry_msgs::PoseArray();
            return true;
        } else {
            return false;
        }
    }

    // TODO: Gap hashing and identification
    bool FGMPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
        path_count ++;
        path_count %= 5;
        traversed_path.header.stamp = ros::Time::now();
        traversed_path.header.frame_id = "map";

        traversed_path.poses.push_back(current_pose_);
        pose_pub.publish(traversed_path);
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

        if (go_to_goal) {
            bool goal_path_clear = checkGoToGoal(goal_angle);
            ROS_DEBUG("Goal Path: %s", goal_path_clear? "Clear":"Not Clear");
            if (goal_path_clear) {
                heading = goal_angle;
                cmd_vel.linear.x = fmin(fabs(0.1 / heading), 0.6);
                // cmd_vel.linear.x = 0;
                heading = fmax(fmin(heading, 0.8), -0.8);
                cmd_vel.angular.z = 0.8 * heading;
                pathVisualization(vis_arr,0,0,0,2);
                return true;
            }
        }

        std::priority_queue <Gap, std::vector<Gap>, gapComparator> pq;        
        // This is a place holder
        Gap currLarge(0,0,0,0,0, goal_angle, score);
        gap_angle = 0;

        // Compute gaps
        bool prev = true; // Take range of pov as wall
        float l_dist = 0;
        float r_dist = 0;
        int size = 0;
        // start_index = 144;
        int start_nan_idx = start_index;
        dmin = 10;

        // Generating Gaps
        // Limited fov
        for(std::vector<float>::size_type it = start_index; it < stored_scan_msgs.ranges.size() && it < 512 - start_index; ++it)
        {
            if (prev) {
                if (stored_scan_msgs.ranges[it] > 2.9) { // isnan
                    ++size;
                } else {
                    r_dist = stored_scan_msgs.ranges[it];
                    dmin = fmin(dmin, r_dist);
                    if (size > 10) {
                        // Filter out noise, tho rarely exists
                        Gap newGap(start_nan_idx, it, size, l_dist, r_dist, goal_angle, score);
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
            // Populate obstacle heredmin
        }

        // Account for error
        if (prev) {
            Gap placeHolder(start_nan_idx, 512 - start_index - 1, size, l_dist, 0, goal_angle, score);        
            pq.push(placeHolder);
        }

        float minD;
        minD = *std::min_element(stored_scan_msgs.ranges.begin(), stored_scan_msgs.ranges.end());

        // Calculate gap angle
        // Do Sticky gap implementation
        // NOT STICKY YET
        if (pq.size() != 0) {
            currLarge = pq.top();
            if (currLarge.getScore() < (lastGap.getScore() / 1.5) && lastGap.getScore() > 0.1) {
                lastGap = currLarge;
                gap_switch_counter = 0;
                lastGap.recordOdom(yaw);
                gap_angle = currLarge.getAngle();
            } else {
                if (gap_switch_counter < 3) {
                    gap_angle = lastGap.getAngle() - lastGap.getOdom() + yaw;
                    gap_switch_counter ++;
                } else {
                    lastGap = currLarge;
                    gap_switch_counter = 0;
                    lastGap.recordOdom(yaw);
                    gap_angle = currLarge.getAngle();
                }
            }
            // gap_angle = currLarge.getAngle();
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

        heading = (alpha / dmin * gap_angle + goal_angle)/(alpha / dmin + 1);
        ROS_DEBUG("Dmin= %f", dmin);

        cmd_vel.linear.x = fmin(fabs(0.1 / heading), max_linear_x) * minD;
        heading = fmax(fmin(heading, max_angular_z), -max_angular_z);
        cmd_vel.angular.z = heading;
        pathVisualization(vis_arr, goal_angle, gap_angle, heading, 0);
        return true;
    }

    FGMPlanner::~FGMPlanner() {
        ROS_INFO_STREAM("Terminated");
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
            if (min_clearance < 0.3) {
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
            if (min_clearance < 0.3) {
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

    void FGMPlanner::pathVisualization(visualization_msgs::MarkerArray vis_arr, float goal_angle, float gap_angle, float heading, int mode) {
        int hash = 0;
        for (int i = 0; i < 40; i++) {
            visualization_msgs::Marker marker;
            // marker. "/base_link"
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time();
            marker.id = hash * 100 + i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = mode;
            marker.pose.position.x = i * 0.05 * cos(goal_angle);
            marker.pose.position.y = i * 0.05 * sin(goal_angle);
            marker.pose.position.z = 0.5;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.lifetime = ros::Duration(0.2);
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            // marker.lifetime = ros::Duration(0.2);
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            vis_arr.markers.push_back(marker);
        }

        hash = 1;
        for (int i = 0; i < 40; i++) {
            visualization_msgs::Marker marker;
            // marker. "/base_link"
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time();
            marker.id = hash * 100 + i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = mode;
            marker.pose.position.x = i * 0.05 * cos(gap_angle);
            marker.pose.position.y = i * 0.05 * sin(gap_angle);
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
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            vis_arr.markers.push_back(marker);
        }

        hash = 2;
        for (int i = 0; i < 40; i++) {
            visualization_msgs::Marker marker;
            // marker. "/base_link"
            marker.header.frame_id = "base_link";
            marker.header.stamp = ros::Time();
            marker.id = hash * 100 + i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = mode;
            marker.pose.position.x = i * 0.05 * cos(heading);
            marker.pose.position.y = i * 0.05 * sin(heading);
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
            marker.color.b = 1.0;
            vis_arr.markers.push_back(marker);
        }
        vis_pub.publish(vis_arr);
        return;
    }

}