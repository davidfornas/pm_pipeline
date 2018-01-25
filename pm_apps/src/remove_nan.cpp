/** 
 * This program is used to remove NaN from PCD.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <iostream>
#include <ros/ros.h>
#include <pm_tools/pcl_tools.h>

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
  PCLTools<PointType>::cloudToPCD(cloud2, std::string(argv[1]) + std::string("_without_nan.pcd"));

  return (0);
}



