/** 
 * This program is used to filter and modify point clouds easily.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <iostream>
#include <ros/ros.h>
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_segmentation.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CPtr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_processing");
  ros::NodeHandle nh;

  CPtr cloud(new Cloud), cloud2(new Cloud);
  PCLTools<PointType>::cloudFromPCD(cloud, std::string(argv[1]) + std::string(".pcd"));
  PCLTools<PointType>::removeNanPoints(cloud, cloud2);
  PCLTools<PointType>::cloudToPCD(cloud2, std::string(argv[1]) + std::string("_processed.pcd"));

  return (0);
}



