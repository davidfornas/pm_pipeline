/** 
 * Test CloudClustering and CloudMeasure
 *
 *  Created on: 04/10/2016
 *      Author: dfornas
 */

#include <pm_tools/pcl_tools.h>
#include <pm_tools/pcl_clustering.h>
#include <pm_perception/cluster_measure.h>
#include <pm_tools/pcl_segmentation.h>

typedef pcl::PointXYZRGB PointT;
typedef typename pcl::PointCloud<PointT> Cloud;
typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "cluster_measure_test");
  ros::NodeHandle nh;

  //Get cloud from topic
  pcl::PointCloud<PointT>::Ptr point_cloud_ptr(new pcl::PointCloud<PointT>);
  PCLTools<PointT>::cloudFromTopic(point_cloud_ptr, "/stereo_down/points2");
  PCLTools<PointT>::removeNanPoints(point_cloud_ptr);

  //Filtering
  PCLTools<PointT>::applyZAxisPassthrough(point_cloud_ptr,0.7, 1.5);
  //PCLTools<PointT>::applyStatisticalOutlierRemoval(point_cloud_ptr, 100, 2);
  PCLTools<PointT>::applyVoxelGridFilter(point_cloud_ptr, 0.01);
  //Remove background
  PlaneSegmentation<PointT>::removeBackground(point_cloud_ptr);

  //Actual Clustering
  CloudClustering<PointT> cluster(point_cloud_ptr);
  cluster.applyEuclidianClustering();
  cluster.displayColoured();
  cluster.save("euclidian");
  /*
  cluster.applyRegionGrowingClustering();
  cluster.displayColoured();
  cluster.save("growing");
  */

  // PCA
  ClusterMeasure<PointT> cm(cluster.cloud_clusters[0]);
  ROS_INFO_STREAM("First cluster centroid: " << cm.getCentroid());
  cm.getAxis();

  Eigen::Quaternionf q;
  Eigen::Vector3f t;
  float width, height, depth;
  cm.getOABBox( q, t, width, height, depth );

  // @TODO Further grasping execution. Maybe separate app

  return 0;
}

