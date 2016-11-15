/** 
 * This program test clustering techniques.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_clustering.h>
#include <pm_perception/cluster_measure.h>

#include <ros/ros.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_clustering_example");
  ros::NodeHandle nh;

  Cloud::Ptr point_cloud_ptr (new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromPCD(point_cloud_ptr, std::string(argv[1])); //Load from PCDReader or from topic

  //Load from UWSim
  //PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, "/stereo_down/points2");

  PCLTools<PointT>::removeNanPoints(point_cloud_ptr);

  ROS_INFO_STREAM("PointCloud loaded without NaN: " << point_cloud_ptr->points.size() << " data points.");
  CloudClustering<PointT> cluster(point_cloud_ptr);

  ROS_INFO_STREAM("CLUSTERING...");

  cluster.applyEuclidianClustering();
  cluster.displayColoured();
  cluster.save("euclidian");
/*
  cluster.applyRegionGrowingClustering();
  cluster.displayColoured();
  cluster.save("growing");


  cluster.applyRGBRegionGrowingClustering();
  cluster.displayColoured();
*/

  //The selection of the cluster can be done using a image of the environment
  // and selecting the corresponding 3D cluster (Toni).


  ClusterMeasure<PointT> cm(cluster.cloud_clusters[0]);
  PCLTools<PointT>::cloudToPCD(cluster.cloud_clusters[0], std::string("fist_cluster.pcd"));
  cm.getCentroid();
  cm.getAxis();

  Eigen::Quaternionf q;
  Eigen::Vector3f t;
  float width, height, depth;

  cm.getOABBox( q, t, width, height, depth );

  /*  PARAMETER TUNNING

  //Iterating to tune parameters... EUCLIDEAN
  for( float i = 0.003; i < 0.025; i += 0.004 ){
	  CloudClustering<PointT> cluster(point_cloud_ptr);
	  cluster.applyEuclidianClustering(i, 400);
	  ROS_INFO_STREAM("Tolerance:" << i);
	  cluster.displayColoured();
  }

  //Iterating to tune parameters... REGION GROWING
  for( float i = 1.4; i < 7.0; i += 0.4 ){
	  CloudClustering<PointT> cluster(point_cloud_ptr);
	  cluster.applyRegionGrowingClustering(5, i, 200);
	  ROS_INFO_STREAM("Tolerance:" << i);
	  cluster.displayColoured();
  }
  */

  return (0);
}

