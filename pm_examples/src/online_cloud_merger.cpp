
/** 
 * This program test the cloud merging utility.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_merge.h>

#include <ros/ros.h>
#include "pcl_ros/point_cloud.h"

#include <pcl/visualization/pcl_visualizer.h>

#define THRESHOLD 100

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_merger_example");
  ros::NodeHandle nh;

  int count=0;

  Cloud::Ptr global_cloud( new Cloud ), new_cloud( new Cloud );

  PCLTools<PointT>::cloudFromTopic(global_cloud, "stereo/points2");
  //double depth = 2, near = 0;
  //nh.getParam("depth", depth);
  //nh.getParam("near", near);
  //Optional: PCLTools::applyZAxisPassthrough(cloud, cloud_filtered, depth, near);
  int last_cloud_count = PCLTools<PointT>::nanAwareCount(global_cloud);
  int cloud_difference = THRESHOLD;
  CloudMerge<PointT> merger(global_cloud);

  // Cloud viewer to watch the cloud as it grows.
  pcl::visualization::PCLVisualizer viewer("Accumulated cloud viewer");

  ros::Rate r(1);
  while (!viewer.wasStopped() && cloud_difference >= THRESHOLD)
  {


    PCLTools<PointT>::cloudFromTopic(new_cloud, "stereo/points2");
	merger.nanAwareOrganizedConcatenateMean(new_cloud);
	int cloud_count = PCLTools<PointT>::nanAwareCount(global_cloud);
    ROS_INFO_STREAM("PointCloud NaN count:" << cloud_count);


    viewer.removePointCloud("Cloud");
    viewer.addPointCloud(global_cloud, "Cloud");
    viewer.spinOnce();

    cloud_difference = cloud_count - last_cloud_count;
    last_cloud_count = cloud_count;
    //Do nothing but wait.
    //r.sleep();
    ROS_INFO_STREAM("PointCloud NaN diff:" << cloud_difference);
  }
  while (!viewer.wasStopped() )
    viewer.spinOnce();
  merger.logResults();
  return (0);
}
