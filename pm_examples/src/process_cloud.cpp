/** 
 * This program is used to filter and modify point clouds easily.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <pm_tools/pcl_tools.h>
#include <ros/ros.h>


typedef pcl::PointXYZRGB PointType;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_processing");
  ros::NodeHandle nh;

  pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
  PCLTools::cloudFromPCD(point_cloud_ptr, std::string(argv[0]) + std::string(".pcd")); //Load from PCDReader or from topic

  ros::Rate r(10);
  while (1)
  {
    //Do nothing but wait.
    r.sleep();
  }

  return (0);
}

