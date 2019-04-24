#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>
#include <ros/ros.h>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "nav_fn_tester");
    ROS_INFO_STREAM("nav_fn tester initiated");

    tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS costmap("my_costmap", tf);

    navfn::NavfnROS navfn;
    navfn.initialize("my_navfn_planner", &costmap);

    return 0;
}