/** 
 * This program test clustering techniques.
 *  Created on: 13/04/2015
 *      Author: dfornas
 */
#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_clustering.h>
#include <pm_perception/cluster_measure.h>

#include <ros/ros.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_clustering_example");
  ros::NodeHandle nh;

  Cloud::Ptr point_cloud_ptr (new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromPCD(point_cloud_ptr, std::string(argv[1]) + std::string(".pcd")); //Load from PCDReader or from topic

  //One time clustering...
  //CloudClustering<PointT> cluster(point_cloud_ptr);

  /*
  cluster.applyEuclidianClustering();
  cluster.displayColoured();
  cluster.save("euclidian");

  cluster.applyRegionGrowingClustering();
  cluster.displayColoured();
  cluster.save("growing");
  */

  CloudClustering<PointT> cluster(point_cloud_ptr);
  cluster.applyRGBRegionGrowingClustering();
  cluster.displayColoured();

  ClusterMeasure<PointT> cm(cluster.cloud_clusters[0]);
  cm.get_centroid();
  cm.get_axis();
  cm.get_bb();

  /*
  //Iterating to tune parameters... EUCLIDEAN
  for( float i = 0.003; i < 0.025; i += 0.004 ){
	  CloudClustering<PointT> cluster(point_cloud_ptr);
	  cluster.applyEuclidianClustering(i, 400);
	  ROS_INFO_STREAM("Tolerance:" << i);
	  cluster.displayColoured();
  }
  */


  /*
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

