/** 
 * This program test RangeImageBorderDetection http://pointclouds.org/documentation/tutorials/range_image_border_extraction.php
 *  Created on: 17/03/2014
 *      Author: dfornas
 */
#include <pm_perception/border_detection.h>

typedef pcl::PointXYZRGB PointT;

int main (int argc, char** argv)
{
  ros::init(argc, argv, "border_following");
  ros::NodeHandle nh;

  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::PointWithRange>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointWithRange>);
  pcl::PCDReader reader;

  reader.read ("uwsimFeb15.pcd", *cloud);
  //Get cloud from topic
  //PCLTools<PointT>::cloudFromTopic(cloud, "/stereo/points2");
  //PCLTools<PointT>::removeNanPoints(cloud);

  RangeImageBorderDetection border_detector(cloud);
  border_detector.process();
  border_detector.getTrajectoryWithRange(cloud_hull);
  ROS_INFO_STREAM("P:" <<cloud_hull->points.size());

  pcl::visualization::PCLVisualizer viewer ("Border detection viewer");
  //viewer.setBackgroundColor (1, 1, 1);
  viewer.addPointCloud(cloud);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> point_cloud_color_handler (cloud_hull, 255, 0, 0);
  viewer.addPointCloud<pcl::PointWithRange> (cloud_hull, point_cloud_color_handler, "points");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "points");

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
    pcl_sleep(0.01);
  }
  return (0);
}
