/** 
 * This program test clustering techniques.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_clustering.h>

#include <ros/ros.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_clustering_example");
  ros::NodeHandle nh;

  Cloud::Ptr point_cloud_ptr (new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromPCD(point_cloud_ptr, std::string(argv[1]) + std::string(".pcd")); //Load from PCDReader or from topic

  CloudClustering<PointT> cluster(point_cloud_ptr);
  cluster.applyEuclidianClustering();

  ros::Rate r(10);
  //Do nothing but wait.
  cluster.display();

  return (0);
}

