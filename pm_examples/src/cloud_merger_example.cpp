/** 
 * This program test the cloud merging utility.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_merge.h>

#include <ros/ros.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGB PointT;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_merger_example");
  ros::NodeHandle nh;

  int clouds_number;
  double depth = 2, near = 0;
  std::string input_basename("cloud");
  nh.getParam("input_basename", input_basename);
  nh.getParam("clouds_number", clouds_number);
  nh.getParam("depth", depth);
  nh.getParam("near", near);
  if (clouds_number < 2)
    return -1;

  //Point Cloud loading and filtering
  std::string point_cloud_file(input_basename + std::string("1.pcd"));
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>), cloud_filtered(new pcl::PointCloud<PointT>), cloud2(
      new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromPCD(cloud, point_cloud_file);
  //Optional: PCLTools::applyZAxisPassthrough(cloud, cloud_filtered, depth, near);

  std::cout << PCLTools<PointT>::nanAwareCount(cloud) << std::endl;
  CloudMerge<PointT> merger(cloud);

  // Cloud viewer to watch the cloud as it grows.
  pcl::visualization::PCLVisualizer viewer("Accumulated cloud");
  for (int i = 2; i <= clouds_number; i++)
  {
    std::ostringstream seq_number;
    seq_number << input_basename << i << std::string(".pcd");
    PCLTools<PointT>::cloudFromPCD(cloud2, seq_number.str());
    merger.nanAwareOrganizedConcatenateMean(cloud2);
    ROS_DEBUG_STREAM("First PointCloud current NaN number:" << PCLTools<PointT>::nanAwareCount(cloud));

    viewer.removePointCloud("Cloud");
    viewer.addPointCloud(cloud, "Cloud");
    viewer.spinOnce();
  }

  pcl::PCDWriter writer;
  writer.write(input_basename + std::string("_out.pcd"), *cloud, false);

  ros::Rate r(10);
  while (!viewer.wasStopped())
  {
    //Do nothing but wait.
    r.sleep();
  }

  return (0);
}

