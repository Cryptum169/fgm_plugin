#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <vector>
#include <queue>
#include <string>
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <fgm_plugin/gap.h>
#include <fgm_plugin/fgm_planner.h>
#include <fgm_plugin/gap_comparator.h>

PLUGINLIB_EXPORT_CLASS(fgm_plugin::FGMPlanner, nav_core::BaseLocalPlanner)

namespace fgm_plugin
{
    FGMPlanner::FGMPlanner() {
        ROS_INFO_STREAM("FGMPlanner planner object created");
        ros::NodeHandle nh;
        // ros::Publisher info_pub;
    }

    void FGMPlanner::laserScanCallback(const sensor_msgs::LaserScan msg) {
        // Copy the value of laser scan message into a local variable
        stored_scan_msgs = msg;
    }

    void FGMPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) {
        // ros::init(argc, argv, "planner_node"); // Node?
        // info_pub = nh.advertise<std::string>("planner_info", 100);
        ROS_INFO_STREAM("FGMPlanner initializde");
        
        // Set up laser scan call back
        ros::Subscriber sub = nh.subscribe("/scan", 1000, &FGMPlanner::laserScanCallback, this);
        ros::spin();
    }

    bool FGMPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
        return true;
    }

    bool FGMPlanner::isGoalReached() {
        return false;
    }

    bool FGMPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
        std::priority_queue <Gap, std::vector<Gap>, gapComparator> pq;
        
        Gap currLarge(0,0,0);
        gap_angle = 0;
        cmd_vel.linear.x = 0.0;

        // Compute gaps
        bool prev = true; // Take range of pov as wall
        int size = 0;
        int start_nan_idx = 0;
        for(std::vector<float>::size_type it = 0; it < stored_scan_msgs.ranges.size(); ++it)
        {
            if (prev) {
                if (stored_scan_msgs.ranges[it] != stored_scan_msgs.ranges[it]) {
                    ++size;
                } else {
                    if (size > 20) {
                        // Be reworked to populate the obstacle size
                        pq.push(Gap(start_nan_idx, size, 0));
                    }
                    start_nan_idx = -1;
                    size = 0;
                    prev = false;
                }
            } else {
                if (stored_scan_msgs.ranges[it] != stored_scan_msgs.ranges[it]) {
                    start_nan_idx = it;
                    size = 1;
                    prev = true;
                }
            }

            // Populate obstacle?
        }

        if (prev) {
            pq.push(Gap(start_nan_idx, size, 0));
        }

        if (pq.size() != 0) {
            currLarge = pq.top();
            gap_angle = (currLarge.getStartAngle() + currLarge.getSize() / 2) * stored_scan_msgs.angle_increment + stored_scan_msgs.angle_min;
            ROS_INFO_STREAM("Gap count:" << pq.size() << " , Bearing: " << gap_angle);
            // std::string info_string = "Gap count:" + std::to_string(pq.size()) + "\n");// << " , Bearing: " << std::to_string(gap_angle) << std::endl;
            // info_pub.publish(info_string);
        }


        
        return true;
    }

    FGMPlanner::~FGMPlanner() {
        ROS_INFO_STREAM("Terminated");
    }
}