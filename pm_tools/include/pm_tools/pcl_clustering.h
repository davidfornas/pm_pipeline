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

  typedef typename pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;

  CloudPtr cloud_;

public:

  std::vector<pcl::PointIndices> cluster_indices;

  CloudClustering(CloudPtr in_cloud) :
      cloud_(in_cloud)
  {
  }

  /** Apply Euclidian clustering http://pointclouds.org/documentation/tutorials/cluster_extraction.php */
  void applyEuclidianClustering();

  /** Apply Conditional Euclidian clustering http://pointclouds.org/documentation/tutorials/conditional_euclidean_clustering.php */
  void applyConditionalEuclidianClustering();

  /** Apply Region Growing clustering pointclouds.org/documentation/tutorials/region_growing_segmentation.php */
  void applyRegionGrowingClustering();

  // ?? pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php

  /** Display result  */
  void display();

};

template<typename PointT>
void CloudClustering<PointT>::display()
{
	/*int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		Cloud::Ptr cloud_cluster (Cloud);
	  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
	  cloud_cluster->points.push_back (cloud_->points[*pit]); //*
	  cloud_cluster->width = cloud_->points.size ();
	  cloud_cluster->height = 1;
	  cloud_cluster->is_dense = true;

	  std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
	  //std::stringstream ss;
	  //ss << "clusters/cloud_cluster_" << j << ".pcd";
	  //writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
	  j++;
	}*/
}


template<typename PointT>
void CloudClustering<PointT>::applyEuclidianClustering()
{
  //std::vector <int> idx;
  //pcl::removeNaNFromPointCloud<Cloud>(*cloud0, *cloud, idx);

  // Creating the KdTree object for the search method of the extraction
  typename pcl::search::KdTree<PointT>::Ptr tree (new typename pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud_);

  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_);
  ec.extract (cluster_indices);
}

template<typename PointT>
void CloudClustering<PointT>::applyConditionalEuclidianClustering()
{

}

template<typename PointT>
void CloudClustering<PointT>::applyRegionGrowingClustering()
{

}

#endif
