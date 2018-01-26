/**
 * Waypoint server for MERBOTS GUI.
 *
 *  Created on: 21/02/2017
 *      Author: dfornas
 */
#include <ros/ros.h>
#include <pm_tools/pcl_tools.h>
#include <pm_tools/marker_tools.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;

/** Marker server
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "Waypoint Server");
  ros::NodeHandle nh;

  WaypointServer wp_server("/dredging_status", "/dredging_pose", nh);
  ros::spin();

  return 0;
}
