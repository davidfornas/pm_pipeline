/**
 * Waypoint server for MERBOTS GUI.
 *
 *  Created on: 21/02/2017
 *      Author: dfornas
 */
#include <ros/ros.h>
#include <pm_manipulation/pm_grasp_planning.h>
#include <pm_tools/pcl_tools.h>
#include <pm_tools/marker_tools.h>


#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;



/** Marker server
 */
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "grasp_server");
  ros::NodeHandle nh;


  WaypointServer wp_server("/dredging_status", "/dredging_pose", nh);

  ros::spin();

  return 0;
}
