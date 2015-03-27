/** 
 * This program test the border detection http://pointclouds.org/documentation/tutorials/range_image_border_extraction.php
 *  Created on: 17/03/2014
 *      Author: dfornas
 */
#include <ros/ros.h>

#include <pm_perception/border_detection.h>

#include <pm_manipulation/trajectory_following.h>

#include <pcl/visualization/pcl_visualizer.h>

//Time measures
#include <ctime>

//typedef pcl::PointXYZRGB PointT;

int
main (int argc, char** argv)
{
  ros::init(argc, argv, "concave_hull_example");
  ros::NodeHandle nh;


  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>), cloud_hull(new pcl::PointCloud<PointT>);
  pcl::PCDReader reader;

  reader.read ("piramide.pcd", *cloud);
//  BorderDetection * border_detector = new ConcaveHullBorderDetection(cloud);
//  border_detector->process();
//  border_detector->getTrajectory(cloud_hull);
  ConcaveHullBorderDetection border_detector(cloud);
  border_detector.process();
  border_detector.getTrajectory(cloud_hull);
  border_detector.generatePath();
  border_detector.transformPathFrame("/world");

  TrajectoryFollowing trajectory_following(border_detector.getPath(), nh, std::string("/uwsim/joint_state"), std::string("/uwsim/joint_state_command"));


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
      std::cout << id;
      viewer.addLine<PointT>(cloud_hull->points[i-1],cloud_hull->points[i],0,255,0,id.str());
    }

  // -----Main loop-----
  //int i=0;
  //viewer.addSphere(cloud_hull->points[i-1], 0.1, )
  long long path_counter=0;
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
    border_detector.publishPath(nh);
    border_detector.publishTF();
    pcl_sleep(0.01);
    path_counter++;
    if(path_counter%100==0)// execute at 1hz
      trajectory_following.moveToNextWaypoint();
  }
  return (0);
}
