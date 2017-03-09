/**
 * Grasp server for MERBOTS GUI.
 *
 *  Created on: 8/02/2017
 *      Author: dfornas
 */
#include <ros/ros.h>
#include <pm_manipulation/pm_grasp_planning.h>
#include <pm_tools/pcl_tools.h>
#include <pm_tools/marker_tools.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> Cloud;

/**
 *  Online grasp planning haciendo el grasp planning continuamente.
 */
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "online_grasp_server");
  ros::NodeHandle nh;

  double angle = 0, rad = 0, along = 0;
  bool alignedGrasp = true;
  std::string input_topic("/input_cloud");
  nh.getParam("input_topic", input_topic);

  //Point Cloud load. Its probably quicker quith a callback
  Cloud::Ptr cloud (new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromTopic(cloud, input_topic);
  PCLTools<PointT>::applyVoxelGridFilter(cloud, 0.001);
  ROS_DEBUG_STREAM("New pointCloud has: " << cloud->points.size() << " data points.");

  //Init planner
  PMGraspPlanning planner(cloud);
  planner.setPlaneSegmentationParams(0.06, 100);

  planner.irad = 2;
  planner.ialong = 2;
  planner.iangle = 2;

  while(ros::ok()){

    //Point Cloud load. Its probably quicker quith a callback
    PCLTools<PointT>::cloudFromTopic(cloud, input_topic);
    PCLTools<PointT>::applyVoxelGridFilter(cloud, 0.001);
    ROS_DEBUG_STREAM("New pointCloud has: " << cloud->points.size() << " data points.");

    //// planner.updateCloud();
    planner.perceive();
    planner.recalculate_cMg();
    vpHomogeneousMatrix cMg = planner.get_cMg();


  }

  return 0;
}
