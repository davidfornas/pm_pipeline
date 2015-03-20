/** 
 * This program test the border detection http://pointclouds.org/documentation/tutorials/range_image_border_extraction.php
 *  Created on: 17/03/2014
 *      Author: dfornas
 */
#include <ros/ros.h>

#include <pm_perception/border_detection.h>



#include <pcl/visualization/pcl_visualizer.h>

//Time measures
#include <ctime>

//typedef pcl::PointXYZRGB PointT;

int
main (int argc, char** argv)
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;


  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>), cloud_hull(new pcl::PointCloud<PointT>);
  pcl::PCDReader reader;

  reader.read ("uwsimFeb15.pcd", *cloud);
//  BorderDetection * border_detector = new ConcaveHullBorderDetection(cloud);
//  border_detector->process();
//  border_detector->getTrajectory(cloud_hull);
  ConcaveHullBorderDetection border_detector(cloud);
  border_detector.process();
  border_detector.getTrajectory(cloud_hull);


  // ----  VISUALIZATION  ---
  // -----Open 3D viewer and add point cloud-----
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor (1, 1, 1);
    //viewer.addCoordinateSystem (1.0f);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> point_cloud_color_handler (cloud, 0, 0, 0);
    viewer.addPointCloud (cloud);//, point_cloud_color_handler, "original point cloud");
    for (int i=1; i<cloud_hull->points.size(); ++i)
    {
      std::ostringstream id;
      id << "name: " << i ;
      viewer.addLine<PointT>(cloud_hull->points[i-1],cloud_hull->points[i],0,255,0,id.str());
    }

  // -----Main loop-----
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
    border_detector.publishMarker(n);
    pcl_sleep(0.01);
  }
  return (0);
}
