/** 
 * This program  is used to maniplation simulation with HOBBIT
 *

 *  Created on: 04/10/2016
 *      Author: dfornas
 */


#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_clustering.h>
#include <pm_perception/cluster_measure.h>
#include <pm_tools/pcl_segmentation.h>
/*
#include <pm_perception/background_removal.h>
#include <pm_perception/object_segmentation.h>

#include <pm_manipulation/pm_grasp_planning.h>
#include <pm_manipulation/hypothesis_generation.h>
#include <pm_manipulation/grasp_hypothesis_evaluation.h>
*/
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

typedef pcl::PointXYZRGB PointT;
typedef typename pcl::PointCloud<PointT> Cloud;
typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "hobbit_grasp_planning_example");
  ros::NodeHandle nh;

  //TODO

  //Get cloud from topic
  pcl::PointCloud<PointT>::Ptr point_cloud_ptr(new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, "/stereo_down/points2");
  PCLTools<PointT>::removeNanPoints(point_cloud_ptr);

  //Maybe some filtering
  PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr,0.7, 1.5);
  //PCLTools<PointT>::applyStatisticalOutlierRemoval(point_cloud_ptr, 100, 2);
  PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);
  //Remove background
  PlaneSegmentation<PointT>::removeBackground(point_cloud_ptr);

  //Clustering

  CloudClustering<PointT> cluster(point_cloud_ptr);
  ROS_INFO_STREAM("CLUSTERING...");

  cluster.applyEuclidianClustering();
  cluster.displayColoured();
  cluster.save("euclidian");
/*
  cluster.applyRegionGrowingClustering();
  cluster.displayColoured();
  cluster.save("growing");*/


  //Doing PCA

  // NOTE The name should be cloud measure OK
  ClusterMeasure<PointT> cm(cluster.cloud_clusters[0]);
  ROS_INFO_STREAM("CENTROID: " << cm.getCentroid());
  cm.getAxis();

  Eigen::Quaternionf q;
  Eigen::Vector3f t;
  float width, height, depth;

  cm.getOABBox( q, t, width, height, depth );

  //Grasping pose stuff
  // For the moment just show the info...



  //Grasping execution. Maybe separade program of course


  return 0;
}

