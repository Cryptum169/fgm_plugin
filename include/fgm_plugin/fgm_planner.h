#ifndef FGM_LOCAL_PLANNER_DWA_PLANNER_H_
#define FGM_LOCAL_PLANNER_DWA_PLANNER_H_


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>


namespace fgm_local_planner {
  /**
   * @class DWAPlanner
   * @brief A class implementing a local planner using the Dynamic Window Approach
   */
    class FGMPlanner : public nav_core::BaseLocalPlann {
        public:
        /**
         * @brief  Constructor for the planner
         * @param name The name of the planner 
         * @param costmap_ros A pointer to the costmap instance the planner should use
         * @param global_frame the frame id of the tf frame to use
         */
        FGMPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util);

        /**
         * @brief  Destructor for the planner
         */
        ~FGMPlanner();

        /**
         * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
         * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
         * @return True if a valid velocity command was found, false otherwise
         */
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) = 0;

        /**
         * @brief  Check if the goal pose has been achieved by the local planner
         * @return True if achieved, false otherwise
         */
        bool isGoalReached() = 0;

        /**
         * @brief  Set the plan that the local planner is following
         * @param plan The plan to pass to the local planner
         * @return True if the plan was updated successfully, false otherwise
         */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) = 0;

        /**
         * @brief  Constructs the local planner
         * @param name The name to give this instance of the local planner
         * @param tf A pointer to a transform listener
         * @param costmap_ros The cost map to use for assigning costs to local plans
         */
        void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) = 0;


    private:
        
  };
};
#endif