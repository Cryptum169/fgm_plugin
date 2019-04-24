#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <math.h>

void laserCallback(const sensor_msgs::LaserScan& msg);
void add_marker(visualization_msgs::MarkerArray& vis_arr, int dir, float degree);
ros::Publisher pub;
ros::Publisher pub_2;
sensor_msgs::LaserScan stored_scan_msgs;
std::vector<int> past_state;
// std::vector<float> angle_boundary;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "level_gap");
    ros::NodeHandle n;

    pub = n.advertise<visualization_msgs::MarkerArray>("/viz_array", 3000);
    pub_2 = n.advertise<visualization_msgs::MarkerArray>("/viz_array_2", 3000);
    ros::Subscriber sub = n.subscribe("/scan",1000, laserCallback);
    past_state = std::vector<int>(640, 1);
    // vector<int> curr_state(640, 1);
    // while (ros::ok()) {
    //     publish_viz(pub);
    // }
    ros::spin();
    return 0;
}

void add_marker(visualization_msgs::MarkerArray& vis_arr, int dir, float degree){
    if (dir == 0) return;
    for (int i = 0; i < 40; i++) {
        visualization_msgs::Marker marker;
        // marker. "/base_link"
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.id = round(i*100 + degree * 10);
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
        marker.lifetime = ros::Duration(0.2);
        marker.color.a = 1.0; // Don't forget to set the alpha!
        if (dir > 0) {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
        } else {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
        }
        marker.color.b = 0.0;
        vis_arr.markers.push_back(marker);
    }
    // ROS_INFO_STREAM(vis_arr.markers.size());
}


void laserCallback(const sensor_msgs::LaserScan& msg) {
    stored_scan_msgs = msg;
    visualization_msgs::MarkerArray vis_arr;
    visualization_msgs::MarkerArray vis_arr_2;
    int this_state = 0;
    int change = 0;
    int count = 0;
    float min = 3;
    float max = -1;
    int min_status = 0;
    int max_status = 0;
    for(std::vector<float>::size_type it = 0; it < stored_scan_msgs.ranges.size(); ++it)
    {
        this_state = stored_scan_msgs.ranges[it] == stored_scan_msgs.ranges[it] ? 1 : -1;

        change = this_state - past_state[it];
        if (change != 0) {
            // ROS_INFO_STREAM(change);
            count += 1;
            float this_angle = (float)(stored_scan_msgs.angle_min + it * stored_scan_msgs.angle_increment);
            add_marker(vis_arr, change, this_angle);

            if (this_angle > max) {
                max = this_angle;
                max_status = change;
            }

            if (this_angle < min) {
                min = this_angle;
                min_status = change;
            }
            // angle_boundary.push_back((float)(stored_scan_msgs.angle_min + it * stored_scan_msgs.angle_increment));
        }
        past_state[it] = this_state;
    }

    // ROS_INFO_STREAM("Max");
    // ROS_INFO_STREAM(max);
    // add_marker(vis_arr, max_status, max);
    // ROS_INFO_STREAM("Min");
    // ROS_INFO_STREAM(min);
    // add_marker(vis_arr_2, min_status, min);
    pub.publish(vis_arr);
    // pub_2.publish(vis_arr_2);
    // vis_arr
}