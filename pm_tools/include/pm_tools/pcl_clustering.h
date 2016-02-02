/*
 * pcl_clustering Description
 *  Created on: 12/05/2015
 *      Author: dfornas
 */
#ifndef PCLCLUSTERING_H_
#define PCLCLUSTERING_H_
#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pm_tools/pcl_tools.h>

template <typename PointT>
class CloudClustering
{

  typedef typename pcl::PointCloud<PointT>::Ptr CloudPtr;
  CloudPtr in_cloud_;

public:

  CloudClustering(CloudPtr in_cloud) :
      in_cloud_(in_cloud)
  {
  }

  /** Apply clustering  */
  void apply();

  /** Display result  */
  void display(){


  }

};



template<typename PointT>
void CloudClustering<PointT>::apply()
{

  CloudPtr cloud0 (new typename pcl::PointCloud<PointT>), cloud (new typename pcl::PointCloud<PointT>), cloud_f (new typename  pcl::PointCloud<PointT>);

  ROS_DEBUG_STREAM("PointCloud before filtering has: " << in_cloud_->points.size () << " data points.");

  //Filtering
  PCLTools<PointT>::applyZAxisPassthrough(in_cloud_, cloud0, 0.2, 1.8);
  std::vector <int> idx;
  pcl::removeNaNFromPointCloud(*cloud0, *cloud, idx);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  typename pcl::VoxelGrid<PointT> vg;
  CloudPtr cloud_filtered (new typename pcl::PointCloud<PointT>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  typename pcl::SACSegmentation<PointT> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  CloudPtr cloud_plane (new typename pcl::PointCloud<PointT> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;

    std::cout << "PointCloud representing cloud filterd: " << cloud_filtered->points.size () << " data points." << std::endl;
  }

  std::cout << "REMAINING " << cloud_filtered->points.size () << " data points." << std::endl;

  writer.write<PointT> ("remaining", *cloud_filtered, false); //*
  // Creating the KdTree object for the search method of the extraction
  typename pcl::search::KdTree<PointT>::Ptr tree (new typename pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
	  CloudPtr cloud_cluster (new typename pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "clusters/cloud_cluster_" << j << ".pcd";
    writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
    j++;
  }
}
#endif
