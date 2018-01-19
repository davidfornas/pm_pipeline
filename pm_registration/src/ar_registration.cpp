/**
 * App: Registration of clouds using ARMarker Poses
 *  Created on: 18/01/2018
 *      Author: dfornas
 */
#include <ros/ros.h>
#include <pm_registration/marker_registration.h>

/*
typedef message_filters::Subscriber<geometry_msgs::PoseStamped> PoseSub;
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> CloudSub;
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::PointCloud2> CloudPoseSync;

typedef pcl::PointXYZRGB PointT;
typedef typename pcl::PointCloud<PointT> Cloud;
typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

typedef pcl::PointNormal NormalT;
typedef typename pcl::PointCloud<NormalT> CloudWithNormals;
typedef typename pcl::PointCloud<NormalT>::Ptr CloudWithNormalsPtr;
*/


int main (int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ar_marker_registration");
  ros::NodeHandle nh;

  // @TODO Add visualization THREADS http://pointclouds.org/documentation/tutorials/cloud_viewer.php
  MarkerRegistration mr(nh, argc, argv);
  mr.run();

  return (0);
}

