#ifndef FGM_LOCAL_PLANNER
#define FGM_LOCAL_PLANNER

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <nav_core/base_local_planner.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>
#include <nav_msgs/Odometry.h>
#include <base_local_planner/local_planner_util.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include "gap.h"
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include <fgm_plugin/FGMConfig.h>

namespace fgm_plugin {
  /**
   * @class DWAPlanner
   * @brief A class implementing a local planner using the Dynamic Window Approach
   */
    class FGMPlanner : public nav_core::BaseLocalPlanner {
        public:
        /**
         * @brief  Constructor for the planner
         * @param name The name of the planner 
         * @param costmap_ros A pointer to the costmap instance the planner should use
         * @param global_frame the frame id of the tf frame to use
         */
        FGMPlanner();

        /**
         * @brief  Destructor for the planner
         */
        ~FGMPlanner();

        /**
         * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid velocity command was found, false otherwise
         */
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        /**
         * @brief  Check if the goal pose has been achieved by the local planner
         * @return True if achieved, false otherwise
         */
        bool isGoalReached();

        /**
         * @brief  Set the plan that the local planner is following
         * @param plan The plan to pass to the local planner
         * @return True if the plan was updated successfully, false otherwise
         */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief  Constructs the local planner
         * @param name The name to give this instance of the local planner
         * @param tf A pointer to a transform listener
         * @param costmap_ros The cost map to use for assigning costs to local plans
         */
        void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

        // Callback functions
        void laserScanCallback(const sensor_msgs::LaserScan msg);
        void poseCallback(const geometry_msgs::Pose msg);
        void reconfigureCb(fgm_plugin::FGMConfig& config, uint32_t level);

        // Utility functions
        bool checkGoToGoal(float goal_angle);
        int angleToSensorIdx(float goal_angle);
        float goalDistance();

        // Visualization
        void pathVisualization(visualization_msgs::MarkerArray vis_arr, float goal_angle, float gap_angle, float heading, int mode);


    private:
        // base_local_planner::LocalPlannerUtil planner_util_;
        std::string planner_name;
        sensor_msgs::LaserScan stored_scan_msgs;
        float gap_angle;
        float goal_angle;
        float heading;
        float dmin;
        float alpha;
        int gap_switch_counter;
        int start_index;
        ros::NodeHandle nh;
        ros::Publisher info_pub;
        ros::Publisher vis_pub;
        ros::Subscriber laser_sub;
        ros::Subscriber pose_sub;

        Gap lastGap;

        geometry_msgs::PoseStamped goal_pose;
        geometry_msgs::Pose current_pose_;
        costmap_2d::Costmap2DROS* costmap_ros_;
        boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser;
        boost::shared_ptr<geometry_msgs::Pose const> sharedPtr_pose;
        base_local_planner::LocalPlannerUtil planner_util_;
        tf::Stamped<tf::Pose> current_drive_cmds;
        // boost::shared_ptr<nav_msgs::Odometry const> sharedPtr_pose;
        // costmap_2d::Costmap2DROS* costmap_ros_;
        tf::Stamped<tf::Pose> current_pose_2;

        boost::shared_ptr<dynamic_reconfigure::Server<fgm_plugin::FGMConfig> > dynamic_recfg_server;
        dynamic_reconfigure::Server<fgm_plugin::FGMConfig>::CallbackType f;

        // Reconfigurable Parameters
        float max_linear_x;
        float max_angular_z;
        float fov;
        bool go_to_goal;
        int sub_goal_idx;
        float goal_distance_tolerance;
  };
};
#endif